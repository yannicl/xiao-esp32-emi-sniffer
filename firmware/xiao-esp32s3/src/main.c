#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "lwip/inet.h"

#include "secrets.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_dsp.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* --------------------------------------------------------------------------
 * Configuration matérielle
 * -------------------------------------------------------------------------- */

/*
 * XIAO ESP32S3
 *
 * D1 = GPIO2 : connecté physiquement à CS de l'ADS7049.
 *              Il est piloté comme MOSI afin de générer le motif CS.
 * D8 = GPIO7 : SCLK
 * D9 = GPIO8 : SDO / MISO
 */
#define ADS7049_PIN_FRAME             GPIO_NUM_2
#define ADS7049_PIN_SCLK              GPIO_NUM_7
#define ADS7049_PIN_SDO               GPIO_NUM_8

#define ADS7049_SPI_HOST              SPI2_HOST
#define ADS7049_SPI_CLOCK_REQUEST_HZ  24000000

/* 17 clocks CS bas + 7 clocks CS haut = 24 clocks par conversion. */
#define BITS_PER_SAMPLE               24U
#define BYTES_PER_SAMPLE              3U

#define SAMPLES_PER_BLOCK             1024U
#define DMA_BLOCK_BYTES               (SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE)
#define DMA_BUFFER_COUNT              2U

#define REPORT_PERIOD_US              10000000LL

#define WIFI_CONNECT_TIMEOUT_MS      10000
#define MQTT_CONNECT_TIMEOUT_MS      7000
#define MQTT_PUBLISH_TIMEOUT_MS      7000
#define MQTT_QOS                     1
#define MQTT_RETAIN                  0
#define MQTT_TOPIC                   "emi/" MQTT_CLIENT_ID "/report"
#define MQTT_PAYLOAD_SIZE            16384

#define ACQUISITION_TASK_CORE         1
#define ACQUISITION_TASK_PRIORITY     18
#define ACQUISITION_TASK_STACK_SIZE   6144

/* --------------------------------------------------------------------------
 * Configuration de l'analyse périodique
 * -------------------------------------------------------------------------- */

#define FFT_SIZE                      1024U
#define FFT_BIN_COUNT                 (FFT_SIZE / 2U)
#define FFT_FIRST_BIN                 1U
#define FFT_LAST_BIN                  (FFT_BIN_COUNT - 1U)

/*
 * Environ 100 FFT par période de 10 secondes.
 * À Fs ~= 1,11 MS/s, un bloc dure ~= 0,922 ms et l'intervalle moyen est
 * donc proche de 100 ms.
 *
 * L'intervalle varie entre 80 et 120 blocs pour éviter de toujours prélever
 * la même phase d'un phénomène périodique lent.
 */
#define FFT_INTERVAL_MIN_BLOCKS       80U
#define FFT_INTERVAL_SPAN_BLOCKS      41U
#define FFT_QUEUE_LENGTH              2U

#define FFT_TASK_CORE                 0
#define FFT_TASK_PRIORITY             10
#define FFT_TASK_STACK_SIZE           8192

#define FFT_POWER_EPSILON             1.0e-30
#define FFT_AMPLITUDE_EPSILON         1.0e-15

/*
 * Plage utile corrigée selon la réponse mesurée du capteur.
 * Les valeurs de puissance sont référées au gain maximal mesuré (0,255 Vrms
 * pour une entrée de 2 Vrms). Il s'agit donc d'une égalisation relative,
 * pas d'une calibration absolue en volts réseau.
 */
#define ANALYSIS_LOWER_HZ             40000.0
#define ANALYSIS_UPPER_HZ             350000.0
#define SENSOR_REFERENCE_OUTPUT_VRMS  0.255

/* Pont résistif ajouté après l'AFE : amplitude divisée par deux.
 * Les niveaux affichés sont ramenés à l'équivalent avant ce pont. */
#define INPUT_ATTENUATION_RATIO       0.5

/* Statistiques par bin : moyenne, maximum et dispersion relative. */
#define ADC_CLIP_LOW_CODE             4U
#define ADC_CLIP_HIGH_CODE            4091U

typedef struct
{
    double frequency_hz;
    double output_vrms;
} sensor_transfer_point_t;

static const sensor_transfer_point_t sensor_transfer_curve[] = {
    {  40000.0, 0.178 },
    {  50000.0, 0.204 },
    {  60000.0, 0.223 },
    {  70000.0, 0.237 },
    {  80000.0, 0.246 },
    {  90000.0, 0.251 },
    { 100000.0, 0.254 },
    { 110000.0, 0.255 },
    { 115000.0, 0.255 },
    { 120000.0, 0.255 },
    { 130000.0, 0.254 },
    { 150000.0, 0.251 },
    { 200000.0, 0.235 },
    { 250000.0, 0.216 },
    { 300000.0, 0.197 },
    { 350000.0, 0.179 },
};

#define SENSOR_TRANSFER_POINT_COUNT \
    ((uint32_t)(sizeof(sensor_transfer_curve) / sizeof(sensor_transfer_curve[0])))

/* Bandes de synthèse couvrant toute la plage 40-350 kHz. */
typedef struct
{
    const char *name;
    double lower_hz;
    double upper_hz;
} frequency_band_definition_t;

static const frequency_band_definition_t frequency_bands[] = {
    { "40-80 kHz",   40000.0,  80000.0 },
    { "80-150 kHz",  80000.0, 150000.0 },
    { "150-250 kHz", 150000.0, 250000.0 },
    { "250-350 kHz", 250000.0, 350000.0 },
};

#define FREQUENCY_BAND_COUNT \
    ((uint32_t)(sizeof(frequency_bands) / sizeof(frequency_bands[0])))

/* --------------------------------------------------------------------------
 * Constantes et objets globaux
 * -------------------------------------------------------------------------- */

static const char *TAG = "ADS7049_DMA";

static spi_device_handle_t adc_spi;
static int actual_spi_frequency_hz = 0;

/* --------------------------------------------------------------------------
 * Statistiques ADC et métriques temporelles
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint16_t minimum;
    uint16_t maximum;

    uint64_t sum;
    uint64_t sample_count;

    uint32_t completed_blocks;
    uint32_t spi_errors;
    uint32_t invalid_samples;
    uint64_t clipped_samples;
    uint32_t clipped_blocks;

    uint64_t block_power_sum;
    uint64_t block_power_minimum;
    uint64_t block_power_maximum;

    uint64_t block_range_sum;
    uint32_t block_range_minimum;
    uint32_t block_range_maximum;

    uint64_t block_diff_sum;
    uint32_t block_diff_minimum;
    uint32_t block_diff_maximum;

    int64_t start_time_us;
} adc_statistics_t;

static adc_statistics_t adc_statistics;

static void adc_statistics_reset(int64_t now_us)
{
    memset(&adc_statistics, 0, sizeof(adc_statistics));

    adc_statistics.minimum = UINT16_MAX;
    adc_statistics.block_power_minimum = UINT64_MAX;
    adc_statistics.block_range_minimum = UINT32_MAX;
    adc_statistics.block_diff_minimum = UINT32_MAX;
    adc_statistics.start_time_us = now_us;
}

/* --------------------------------------------------------------------------
 * Buffers DMA
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    spi_transaction_t transaction;
} dma_buffer_t;

static dma_buffer_t dma_buffers[DMA_BUFFER_COUNT];

/* --------------------------------------------------------------------------
 * Données FFT
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint16_t samples[FFT_SIZE];
} fft_job_t;

typedef struct
{
    uint64_t requested;
    uint64_t queued;
    uint64_t completed;
    uint64_t dropped;
    uint64_t errors;

    uint64_t total_time_us;
    uint32_t maximum_time_us;

    double input_mean_sum;
    double input_rms_sum;
    double input_rms_minimum;
    double input_rms_maximum;

    /* Statistiques par bin, accumulées en puissance linéaire corrigée. */
    double bin_power_sum[FFT_BIN_COUNT];
    double bin_power_squared_sum[FFT_BIN_COUNT];
    double bin_power_minimum[FFT_BIN_COUNT];
    double bin_power_maximum[FFT_BIN_COUNT];

    /* Statistiques des bandes : données dérivées de la FFT, pour compatibilité
     * avec le dashboard et pour les indicateurs synthétiques. */
    double band_power_sum[FREQUENCY_BAND_COUNT];
    double band_power_squared_sum[FREQUENCY_BAND_COUNT];
    double band_power_minimum[FREQUENCY_BAND_COUNT];
    double band_power_maximum[FREQUENCY_BAND_COUNT];

    double total_analyzed_power_sum;
    double centroid_hz_sum;
    double flatness_sum;

    uint64_t dominant_bin_histogram[FFT_BIN_COUNT];
    uint32_t strongest_instantaneous_bin;
    double strongest_instantaneous_power;
} fft_statistics_t;

static QueueHandle_t fft_job_queue;
static SemaphoreHandle_t fft_statistics_mutex;

static fft_statistics_t fft_statistics;
static fft_statistics_t fft_report_snapshot;

static float fft_window[FFT_SIZE] __attribute__((aligned(16)));
static float fft_data[2U * FFT_SIZE] __attribute__((aligned(16)));
static double fft_bin_power_scratch[FFT_BIN_COUNT] __attribute__((aligned(16)));

static double fft_window_sum = 0.0;
static double fft_window_squared_sum = 0.0;

/* État du planificateur de prélèvements FFT. */
static uint32_t fft_blocks_until_capture = 1U;
static uint32_t fft_prng_state = 0x6D2B79F5U;

static uint32_t next_fft_random_u32(void)
{
    /* xorshift32 : très léger et suffisant pour déphaser les prélèvements. */
    uint32_t value = fft_prng_state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    fft_prng_state = value;
    return value;
}

static uint32_t choose_next_fft_interval(void)
{
    return FFT_INTERVAL_MIN_BLOCKS +
           (next_fft_random_u32() % FFT_INTERVAL_SPAN_BLOCKS);
}

static void fft_statistics_reset_locked(void)
{
    memset(&fft_statistics, 0, sizeof(fft_statistics));

    fft_statistics.input_rms_minimum = HUGE_VAL;

    for (uint32_t bin = 0; bin < FFT_BIN_COUNT; bin++)
    {
        fft_statistics.bin_power_minimum[bin] = HUGE_VAL;
    }

    for (uint32_t band = 0; band < FREQUENCY_BAND_COUNT; band++)
    {
        fft_statistics.band_power_minimum[band] = HUGE_VAL;
    }
}

static void take_fft_statistics_snapshot(void)
{
    xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
    memcpy(&fft_report_snapshot, &fft_statistics, sizeof(fft_report_snapshot));
    fft_statistics_reset_locked();
    xSemaphoreGive(fft_statistics_mutex);
}

static void wait_for_fft_jobs_to_finish(void)
{
    const int64_t deadline_us = esp_timer_get_time() + 250000LL;

    while (esp_timer_get_time() < deadline_us)
    {
        uint64_t queued;
        uint64_t finished;

        xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
        queued = fft_statistics.queued;
        finished = fft_statistics.completed + fft_statistics.errors;
        xSemaphoreGive(fft_statistics_mutex);

        if (finished >= queued)
        {
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGW(TAG, "Timeout en attendant la fin des FFT avant le rapport");
}

/* --------------------------------------------------------------------------
 * Motif CS transmis sur MOSI
 * -------------------------------------------------------------------------- */

static void initialize_frame_pattern(uint8_t *buffer)
{
    /* Motif MSB first : 00000000 00000000 01111111. */
    for (uint32_t sample = 0; sample < SAMPLES_PER_BLOCK; sample++)
    {
        const uint32_t offset = sample * BYTES_PER_SAMPLE;
        buffer[offset + 0U] = 0x00;
        buffer[offset + 1U] = 0x00;
        buffer[offset + 2U] = 0x7F;
    }
}

/* --------------------------------------------------------------------------
 * Décodage ADS7049
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint16_t value;
    bool framing_valid;
} decoded_sample_t;

static inline decoded_sample_t decode_sample(
    const uint8_t *rx_buffer,
    uint32_t sample_index)
{
    const uint32_t offset = sample_index * BYTES_PER_SAMPLE;

    const uint16_t raw =
        ((uint16_t)rx_buffer[offset + 0U] << 8) |
        ((uint16_t)rx_buffer[offset + 1U]);

    decoded_sample_t result = {
        .value = (raw >> 2) & 0x0FFFU,
        .framing_valid = (raw & 0xC003U) == 0U
    };

    return result;
}

/* --------------------------------------------------------------------------
 * Outils spectraux
 * -------------------------------------------------------------------------- */

static uint32_t frequency_to_first_bin(double frequency_hz, double sample_rate_hz)
{
    uint32_t bin = (uint32_t)ceil(frequency_hz * FFT_SIZE / sample_rate_hz);

    if (bin < FFT_FIRST_BIN)
    {
        bin = FFT_FIRST_BIN;
    }
    if (bin > FFT_LAST_BIN)
    {
        bin = FFT_LAST_BIN;
    }

    return bin;
}

static uint32_t frequency_to_last_bin(double frequency_hz, double sample_rate_hz)
{
    uint32_t bin = (uint32_t)floor(frequency_hz * FFT_SIZE / sample_rate_hz);

    if (bin < FFT_FIRST_BIN)
    {
        bin = FFT_FIRST_BIN;
    }
    if (bin > FFT_LAST_BIN)
    {
        bin = FFT_LAST_BIN;
    }

    return bin;
}

static double spectral_power_to_rms_fs(double positive_bin_power)
{
    /*
     * Parseval, spectre simple face et correction de l'énergie de Hann :
     * RMS^2 ~= 2 * somme(|X[k]|^2) / (N * somme(w[n]^2)).
     */
    const double rms_squared =
        2.0 * positive_bin_power /
        ((double)FFT_SIZE * fft_window_squared_sum);

    return sqrt(fmax(rms_squared, 0.0));
}

static double rms_fs_to_dbfs(double rms_fs)
{
    return 20.0 * log10(fmax(rms_fs, FFT_AMPLITUDE_EPSILON));
}


static double sensor_output_vrms_at_frequency(double frequency_hz)
{
    if (frequency_hz <= sensor_transfer_curve[0].frequency_hz)
    {
        return sensor_transfer_curve[0].output_vrms;
    }

    for (uint32_t index = 1U; index < SENSOR_TRANSFER_POINT_COUNT; index++)
    {
        if (frequency_hz <= sensor_transfer_curve[index].frequency_hz)
        {
            const sensor_transfer_point_t *lower = &sensor_transfer_curve[index - 1U];
            const sensor_transfer_point_t *upper = &sensor_transfer_curve[index];
            const double fraction =
                (frequency_hz - lower->frequency_hz) /
                (upper->frequency_hz - lower->frequency_hz);

            return lower->output_vrms +
                   fraction * (upper->output_vrms - lower->output_vrms);
        }
    }

    return sensor_transfer_curve[SENSOR_TRANSFER_POINT_COUNT - 1U].output_vrms;
}

static double sensor_power_correction(double frequency_hz)
{
    const double measured_output = sensor_output_vrms_at_frequency(frequency_hz);
    const double amplitude_correction =
        SENSOR_REFERENCE_OUTPUT_VRMS / fmax(measured_output, 1.0e-9);
    const double sensor_correction =
        amplitude_correction * amplitude_correction;
    const double attenuation_power =
        INPUT_ATTENUATION_RATIO * INPUT_ATTENUATION_RATIO;

    return sensor_correction / fmax(attenuation_power, 1.0e-12);
}

/* --------------------------------------------------------------------------
 * Traitement FFT
 * -------------------------------------------------------------------------- */

static esp_err_t process_fft_job(const fft_job_t *job)
{
    double input_sum = 0.0;

    for (uint32_t index = 0; index < FFT_SIZE; index++)
    {
        input_sum += job->samples[index];
    }

    const double input_mean = input_sum / (double)FFT_SIZE;
    double squared_sum = 0.0;

    for (uint32_t index = 0; index < FFT_SIZE; index++)
    {
        const float centered =
            ((float)job->samples[index] - (float)input_mean) / 2048.0f;

        squared_sum += (double)centered * (double)centered;
        fft_data[2U * index] = centered * fft_window[index];
        fft_data[2U * index + 1U] = 0.0f;
    }

    const double input_rms = sqrt(squared_sum / (double)FFT_SIZE);
    const int64_t start_us = esp_timer_get_time();

    esp_err_t result = dsps_fft2r_fc32(fft_data, FFT_SIZE);
    if (result != ESP_OK)
    {
        return result;
    }

    result = dsps_bit_rev_fc32(fft_data, FFT_SIZE);
    if (result != ESP_OK)
    {
        return result;
    }

    const uint32_t elapsed_us =
        (uint32_t)(esp_timer_get_time() - start_us);

    const double sample_rate_hz =
        (double)actual_spi_frequency_hz / (double)BITS_PER_SAMPLE;
    const double frequency_resolution_hz = sample_rate_hz / FFT_SIZE;

    memset(fft_bin_power_scratch, 0, sizeof(fft_bin_power_scratch));

    double total_analyzed_power = 0.0;
    double weighted_frequency_power = 0.0;
    double logarithmic_power_sum = 0.0;
    uint32_t analyzed_bin_count = 0U;

    uint32_t dominant_bin = FFT_FIRST_BIN;
    double dominant_power = 0.0;

    const uint32_t analyzed_first_bin =
        frequency_to_first_bin(ANALYSIS_LOWER_HZ, sample_rate_hz);
    const uint32_t analyzed_last_bin =
        frequency_to_last_bin(ANALYSIS_UPPER_HZ, sample_rate_hz);

    for (uint32_t bin = FFT_FIRST_BIN; bin <= FFT_LAST_BIN; bin++)
    {
        const double real = fft_data[2U * bin];
        const double imaginary = fft_data[2U * bin + 1U];
        const double raw_power = real * real + imaginary * imaginary;
        const double frequency_hz = bin * frequency_resolution_hz;
        const double power =
            (frequency_hz >= ANALYSIS_LOWER_HZ && frequency_hz <= ANALYSIS_UPPER_HZ)
                ? raw_power * sensor_power_correction(frequency_hz)
                : 0.0;

        fft_bin_power_scratch[bin] = power;

        if (bin >= analyzed_first_bin && bin <= analyzed_last_bin)
        {
            total_analyzed_power += power;
            weighted_frequency_power += frequency_hz * power;
            logarithmic_power_sum += log(fmax(power, FFT_POWER_EPSILON));
            analyzed_bin_count++;

            if (power > dominant_power)
            {
                dominant_power = power;
                dominant_bin = bin;
            }
        }
    }

    double band_power[FREQUENCY_BAND_COUNT];
    memset(band_power, 0, sizeof(band_power));

    for (uint32_t band = 0; band < FREQUENCY_BAND_COUNT; band++)
    {
        const uint32_t first_bin = frequency_to_first_bin(
            frequency_bands[band].lower_hz, sample_rate_hz);
        const uint32_t last_bin = frequency_to_last_bin(
            frequency_bands[band].upper_hz, sample_rate_hz);

        for (uint32_t bin = first_bin; bin <= last_bin; bin++)
        {
            band_power[band] += fft_bin_power_scratch[bin];
        }
    }

    const double centroid_hz =
        weighted_frequency_power /
        fmax(total_analyzed_power, FFT_POWER_EPSILON);

    const double arithmetic_mean_power =
        total_analyzed_power / fmax((double)analyzed_bin_count, 1.0);

    const double geometric_mean_power =
        exp(logarithmic_power_sum /
            fmax((double)analyzed_bin_count, 1.0));

    const double flatness =
        geometric_mean_power /
        fmax(arithmetic_mean_power, FFT_POWER_EPSILON);

    xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);

    fft_statistics.completed++;
    fft_statistics.total_time_us += elapsed_us;
    fft_statistics.input_mean_sum += input_mean;
    fft_statistics.input_rms_sum += input_rms;

    if (input_rms < fft_statistics.input_rms_minimum)
        fft_statistics.input_rms_minimum = input_rms;
    if (input_rms > fft_statistics.input_rms_maximum)
        fft_statistics.input_rms_maximum = input_rms;
    if (elapsed_us > fft_statistics.maximum_time_us)
        fft_statistics.maximum_time_us = elapsed_us;

    /* Statistiques par bin : moyenne, dispersion et maximum de puissance. */
    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        const double power = fft_bin_power_scratch[bin];
        fft_statistics.bin_power_sum[bin] += power;
        fft_statistics.bin_power_squared_sum[bin] += power * power;

        if (power < fft_statistics.bin_power_minimum[bin])
            fft_statistics.bin_power_minimum[bin] = power;
        if (power > fft_statistics.bin_power_maximum[bin])
            fft_statistics.bin_power_maximum[bin] = power;
    }

    /* Bandes conservées uniquement comme métriques dérivées. */
    for (uint32_t band = 0; band < FREQUENCY_BAND_COUNT; band++)
    {
        const double power = band_power[band];
        fft_statistics.band_power_sum[band] += power;
        fft_statistics.band_power_squared_sum[band] += power * power;

        if (power < fft_statistics.band_power_minimum[band])
            fft_statistics.band_power_minimum[band] = power;
        if (power > fft_statistics.band_power_maximum[band])
            fft_statistics.band_power_maximum[band] = power;
    }

    fft_statistics.total_analyzed_power_sum += total_analyzed_power;
    fft_statistics.centroid_hz_sum += centroid_hz;
    fft_statistics.flatness_sum += flatness;
    fft_statistics.dominant_bin_histogram[dominant_bin]++;

    if (dominant_power > fft_statistics.strongest_instantaneous_power)
    {
        fft_statistics.strongest_instantaneous_power = dominant_power;
        fft_statistics.strongest_instantaneous_bin = dominant_bin;
    }

    xSemaphoreGive(fft_statistics_mutex);

    return ESP_OK;
}

static void fft_task(void *argument)
{
    (void)argument;
    fft_job_t job;

    while (true)
    {
        if (xQueueReceive(fft_job_queue, &job, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        const esp_err_t result = process_fft_job(&job);

        if (result != ESP_OK)
        {
            xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
            fft_statistics.errors++;
            xSemaphoreGive(fft_statistics_mutex);

            ESP_LOGE(TAG, "Erreur FFT: %s", esp_err_to_name(result));
        }
    }
}

/* --------------------------------------------------------------------------
 * Traitement d'un bloc DMA
 * -------------------------------------------------------------------------- */

static void process_completed_block(const dma_buffer_t *buffer)
{
    uint16_t block_minimum = UINT16_MAX;
    uint16_t block_maximum = 0U;

    uint64_t block_sum = 0U;
    uint64_t block_squared_sum = 0U;
    uint64_t block_absolute_difference_sum = 0U;
    uint32_t block_invalid_samples = 0U;
    uint32_t block_clipped_samples = 0U;

    uint16_t previous_sample = 0U;
    bool previous_sample_valid = false;

    const bool capture_fft = (fft_blocks_until_capture <= 1U);
    fft_job_t fft_job;

    for (uint32_t index = 0; index < SAMPLES_PER_BLOCK; index++)
    {
        const decoded_sample_t decoded =
            decode_sample(buffer->rx_buffer, index);
        const uint16_t sample = decoded.value;

        if (!decoded.framing_valid)
        {
            block_invalid_samples++;
        }

        if (sample <= ADC_CLIP_LOW_CODE || sample >= ADC_CLIP_HIGH_CODE)
        {
            block_clipped_samples++;
        }

        if (sample < block_minimum)
        {
            block_minimum = sample;
        }
        if (sample > block_maximum)
        {
            block_maximum = sample;
        }

        block_sum += sample;
        block_squared_sum += (uint64_t)sample * (uint64_t)sample;

        if (previous_sample_valid)
        {
            block_absolute_difference_sum +=
                (sample >= previous_sample)
                    ? (uint32_t)(sample - previous_sample)
                    : (uint32_t)(previous_sample - sample);
        }

        previous_sample = sample;
        previous_sample_valid = true;

        if (capture_fft)
        {
            fft_job.samples[index] = sample;
        }
    }

    const uint32_t block_mean =
        (uint32_t)(block_sum / SAMPLES_PER_BLOCK);

    const uint64_t mean_square =
        block_squared_sum / SAMPLES_PER_BLOCK;
    const uint64_t squared_mean =
        (uint64_t)block_mean * (uint64_t)block_mean;
    const uint64_t block_ac_power =
        (mean_square >= squared_mean) ? (mean_square - squared_mean) : 0U;

    const uint32_t block_range = block_maximum - block_minimum;
    const uint32_t block_mean_absolute_difference =
        (uint32_t)(block_absolute_difference_sum /
                   (SAMPLES_PER_BLOCK - 1U));

    if (block_minimum < adc_statistics.minimum)
    {
        adc_statistics.minimum = block_minimum;
    }
    if (block_maximum > adc_statistics.maximum)
    {
        adc_statistics.maximum = block_maximum;
    }

    adc_statistics.sum += block_sum;
    adc_statistics.sample_count += SAMPLES_PER_BLOCK;
    adc_statistics.invalid_samples += block_invalid_samples;
    adc_statistics.clipped_samples += block_clipped_samples;
    if (block_clipped_samples > 0U)
    {
        adc_statistics.clipped_blocks++;
    }
    adc_statistics.completed_blocks++;

    adc_statistics.block_power_sum += block_ac_power;
    adc_statistics.block_range_sum += block_range;
    adc_statistics.block_diff_sum += block_mean_absolute_difference;

    if (block_ac_power < adc_statistics.block_power_minimum)
    {
        adc_statistics.block_power_minimum = block_ac_power;
    }
    if (block_ac_power > adc_statistics.block_power_maximum)
    {
        adc_statistics.block_power_maximum = block_ac_power;
    }
    if (block_range < adc_statistics.block_range_minimum)
    {
        adc_statistics.block_range_minimum = block_range;
    }
    if (block_range > adc_statistics.block_range_maximum)
    {
        adc_statistics.block_range_maximum = block_range;
    }
    if (block_mean_absolute_difference < adc_statistics.block_diff_minimum)
    {
        adc_statistics.block_diff_minimum = block_mean_absolute_difference;
    }
    if (block_mean_absolute_difference > adc_statistics.block_diff_maximum)
    {
        adc_statistics.block_diff_maximum = block_mean_absolute_difference;
    }

    if (capture_fft)
    {
        xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
        fft_statistics.requested++;
        xSemaphoreGive(fft_statistics_mutex);

        if (xQueueSend(fft_job_queue, &fft_job, 0) == pdTRUE)
        {
            xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
            fft_statistics.queued++;
            xSemaphoreGive(fft_statistics_mutex);
        }
        else
        {
            xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
            fft_statistics.dropped++;
            xSemaphoreGive(fft_statistics_mutex);
        }

        fft_blocks_until_capture = choose_next_fft_interval();
    }
    else
    {
        fft_blocks_until_capture--;
    }
}

/* --------------------------------------------------------------------------
 * Rapport FFT représentant la période
 * -------------------------------------------------------------------------- */

static void print_fft_report(double sample_rate_hz)
{
    const fft_statistics_t *snapshot = &fft_report_snapshot;

    if (snapshot->completed == 0U)
    {
        ESP_LOGW(TAG, "Profil 10 s: aucune FFT terminée");
        return;
    }

    const double completed = (double)snapshot->completed;
    const double resolution_hz = sample_rate_hz / FFT_SIZE;

    const uint32_t analyzed_first_bin =
        frequency_to_first_bin(ANALYSIS_LOWER_HZ, sample_rate_hz);
    const uint32_t analyzed_last_bin =
        frequency_to_last_bin(ANALYSIS_UPPER_HZ, sample_rate_hz);

    uint32_t dominant_bin = analyzed_first_bin;
    double dominant_power = 0.0;
    double total_power = 0.0;

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        const double mean_power = snapshot->bin_power_sum[bin] / completed;
        total_power += mean_power;
        if (mean_power > dominant_power)
        {
            dominant_power = mean_power;
            dominant_bin = bin;
        }
    }

    const double dominant_peak_amplitude =
        2.0 * sqrt(fmax(dominant_power, FFT_POWER_EPSILON)) / fft_window_sum;
    const double strongest_peak_amplitude =
        2.0 * sqrt(fmax(snapshot->strongest_instantaneous_power,
                        FFT_POWER_EPSILON)) / fft_window_sum;

    ESP_LOGI(TAG,
             "Profil spectral 10 s: FFT=%" PRIu64
             ", perdues=%" PRIu64 ", erreurs=%" PRIu64,
             snapshot->completed, snapshot->dropped, snapshot->errors);

    ESP_LOGI(TAG,
             "FFT: moyen=%.1f us, max=%" PRIu32
             " us, résolution=%.2f Hz/bin",
             (double)snapshot->total_time_us / completed,
             snapshot->maximum_time_us,
             resolution_hz);

    ESP_LOGI(TAG,
             "Spectre corrigé %.0f-%.0f kHz: dominante=%.1f kHz, %.2f dBFS crête"
             ", puissance totale=%.2f dB",
             ANALYSIS_LOWER_HZ / 1000.0,
             ANALYSIS_UPPER_HZ / 1000.0,
             dominant_bin * resolution_hz / 1000.0,
             20.0 * log10(fmax(dominant_peak_amplitude, FFT_AMPLITUDE_EPSILON)),
             10.0 * log10(fmax(total_power, FFT_POWER_EPSILON)));

    ESP_LOGI(TAG,
             "Pic instantané maximal: %.1f kHz, %.2f dBFS crête"
             ", centroïde moyen=%.1f kHz, flatness=%.3f",
             snapshot->strongest_instantaneous_bin * resolution_hz / 1000.0,
             20.0 * log10(fmax(strongest_peak_amplitude, FFT_AMPLITUDE_EPSILON)),
             (snapshot->centroid_hz_sum / completed) / 1000.0,
             snapshot->flatness_sum / completed);

    /* Les bandes sont maintenant une synthèse secondaire de la FFT complète. */
    for (uint32_t band = 0; band < FREQUENCY_BAND_COUNT; band++)
    {
        const double mean_power = snapshot->band_power_sum[band] / completed;
        const double mean_power_squared =
            snapshot->band_power_squared_sum[band] / completed;
        const double variance =
            fmax(mean_power_squared - mean_power * mean_power, 0.0);
        const double variability_percent =
            100.0 * sqrt(variance) / fmax(mean_power, FFT_POWER_EPSILON);

        ESP_LOGI(TAG,
                 "Bande %-11s: %.2f dBFS RMS, variabilité=%.1f %%",
                 frequency_bands[band].name,
                 rms_fs_to_dbfs(spectral_power_to_rms_fs(mean_power)),
                 variability_percent);
    }
}

/* --------------------------------------------------------------------------
 * Wi-Fi / MQTT
 *
 * Repris de main_ref.c : la pile réseau est initialisée une seule fois, mais
 * le Wi-Fi reste arrêté pendant les mesures. Il n'est activé que le temps
 * d'envoyer un rapport MQTT, puis est complètement arrêté avant de reprendre
 * l'acquisition.
 * -------------------------------------------------------------------------- */

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT0
#define MQTT_PUBLISHED_BIT BIT1

static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t mqtt_event_group;
static esp_netif_t *wifi_netif = NULL;
static bool wifi_initialized = false;

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool parse_ipv4(const char *text, esp_ip4_addr_t *out)
{
    ip4_addr_t tmp;

    if (!ip4addr_aton(text, &tmp))
    {
        return false;
    }

    out->addr = tmp.addr;
    return true;
}

static void log_heap(const char *where)
{
    ESP_LOGI(
        TAG,
        "Heap %s: libre=%lu octets, plus grand bloc=%lu, minimum historique=%lu",
        where,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
        (unsigned long)esp_get_minimum_free_heap_size());
}

static void init_network_once(void)
{
    if (wifi_initialized)
    {
        return;
    }

    esp_err_t result = nvs_flash_init();

    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(result);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_event_group = xEventGroupCreate();
    mqtt_event_group = xEventGroupCreate();

    if (wifi_event_group == NULL || mqtt_event_group == NULL)
    {
        ESP_LOGE(TAG, "Allocation des EventGroups réseau impossible");
        abort();
    }

    wifi_netif = esp_netif_create_default_wifi_sta();

    if (wifi_netif == NULL)
    {
        ESP_LOGE(TAG, "Création de l'interface Wi-Fi STA impossible");
        abort();
    }

#if USE_STATIC_IP
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(wifi_netif));

    esp_netif_ip_info_t ip_info = {0};
    esp_netif_dns_info_t dns_info = {0};

    if (!parse_ipv4(WIFI_IP_ADDRESS, &ip_info.ip) ||
        !parse_ipv4(WIFI_GATEWAY, &ip_info.gw) ||
        !parse_ipv4(WIFI_SUBNET, &ip_info.netmask) ||
        !parse_ipv4(WIFI_DNS, &dns_info.ip.u_addr.ip4))
    {
        ESP_LOGE(TAG, "Configuration IPv4 statique invalide dans secrets.h");
        abort();
    }

    dns_info.ip.type = ESP_IPADDR_TYPE_V4;

    ESP_ERROR_CHECK(esp_netif_set_ip_info(wifi_netif, &ip_info));
    ESP_ERROR_CHECK(
        esp_netif_set_dns_info(
            wifi_netif,
            ESP_NETIF_DNS_MAIN,
            &dns_info));
#endif

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL));

    wifi_config_t wifi_config = {0};

    strlcpy(
        (char *)wifi_config.sta.ssid,
        WIFI_SSID,
        sizeof(wifi_config.sta.ssid));

    strlcpy(
        (char *)wifi_config.sta.password,
        WIFI_PASSWORD,
        sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    wifi_initialized = true;

    ESP_LOGI(
        TAG,
        "Wi-Fi initialisé; radio arrêtée pendant les acquisitions");
}

static bool wifi_connect_for_publish(void)
{
    init_network_once();

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    esp_err_t result = esp_wifi_start();

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Démarrage Wi-Fi impossible: %s",
            esp_err_to_name(result));
        return false;
    }

    result = esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);

    if (result != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Réglage puissance Wi-Fi impossible: %s",
            esp_err_to_name(result));
    }

    const EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) == 0U)
    {
        ESP_LOGE(TAG, "Timeout de connexion Wi-Fi");
        esp_wifi_stop();
        return false;
    }

    ESP_LOGI(TAG, "Wi-Fi connecté pour publication MQTT");
    return true;
}

static void wifi_stop_after_publish(void)
{
    if (!wifi_initialized)
    {
        return;
    }

    esp_wifi_disconnect();

    const esp_err_t result = esp_wifi_stop();

    if (result != ESP_OK && result != ESP_ERR_WIFI_NOT_STARTED)
    {
        ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(result));
    }

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "Wi-Fi arrêté; reprise des mesures sans RF Wi-Fi");
}

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_args;
    (void)base;

    const esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;

    if (event_id == MQTT_EVENT_CONNECTED)
    {
        xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);
    }
    else if (event_id == MQTT_EVENT_DISCONNECTED)
    {
        xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
    }
    else if (event_id == MQTT_EVENT_PUBLISHED)
    {
        xEventGroupSetBits(mqtt_event_group, MQTT_PUBLISHED_BIT);
    }
    else if (event_id == MQTT_EVENT_ERROR)
    {
        ESP_LOGW(TAG, "Erreur de transport MQTT");
    }

    (void)event;
}

static bool json_append(
    char *buffer,
    size_t buffer_size,
    size_t *used,
    const char *format,
    ...)
{
    if (*used >= buffer_size)
    {
        return false;
    }

    va_list arguments;
    va_start(arguments, format);

    const int length = vsnprintf(
        buffer + *used,
        buffer_size - *used,
        format,
        arguments);

    va_end(arguments);

    if (length < 0 || (size_t)length >= buffer_size - *used)
    {
        buffer[buffer_size - 1U] = '\0';
        return false;
    }

    *used += (size_t)length;
    return true;
}

static bool build_mqtt_payload(
    char *payload,
    size_t payload_size,
    int64_t elapsed_us,
    double sample_rate_hz)
{
    const fft_statistics_t *snapshot = &fft_report_snapshot;

    if (elapsed_us <= 0 ||
        adc_statistics.sample_count == 0U ||
        snapshot->completed == 0U)
    {
        return false;
    }

    size_t used = 0U;
    const double elapsed_seconds = (double)elapsed_us / 1000000.0;
    const double completed = (double)snapshot->completed;
    const double resolution_hz = sample_rate_hz / FFT_SIZE;

    const uint32_t analyzed_first_bin =
        frequency_to_first_bin(ANALYSIS_LOWER_HZ, sample_rate_hz);
    const uint32_t analyzed_last_bin =
        frequency_to_last_bin(ANALYSIS_UPPER_HZ, sample_rate_hz);

    uint32_t dominant_bin = analyzed_first_bin;
    double dominant_power = 0.0;
    double total_power = 0.0;

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        const double mean_power = snapshot->bin_power_sum[bin] / completed;
        total_power += mean_power;
        if (mean_power > dominant_power)
        {
            dominant_power = mean_power;
            dominant_bin = bin;
        }
    }

    const double dominant_peak_amplitude =
        2.0 * sqrt(fmax(dominant_power, FFT_POWER_EPSILON)) / fft_window_sum;
    const double strongest_peak_amplitude =
        2.0 * sqrt(fmax(snapshot->strongest_instantaneous_power,
                        FFT_POWER_EPSILON)) / fft_window_sum;

    const double block_count =
        fmax((double)adc_statistics.completed_blocks, 1.0);
    const double adc_average =
        (double)adc_statistics.sum / (double)adc_statistics.sample_count;
    const double effective_sample_rate =
        (double)adc_statistics.sample_count / elapsed_seconds;

    if (!json_append(payload, payload_size, &used,
                     "{\"client_id\":\"%s\"," 
                     "\"period_s\":%.6f," 
                     "\"spi_hz\":%d," 
                     "\"sample_rate_hz\":%.3f," 
                     "\"effective_sample_rate_hz\":%.3f,"
                     "\"adc\":{" 
                     "\"blocks\":%" PRIu32 "," 
                     "\"samples\":%" PRIu64 "," 
                     "\"spi_errors\":%" PRIu32 "," 
                     "\"invalid_samples\":%" PRIu32 "," 
                     "\"clipped_samples\":%" PRIu64 "," 
                     "\"clipped_blocks\":%" PRIu32 "," 
                     "\"min_code\":%" PRIu16 "," 
                     "\"max_code\":%" PRIu16 "," 
                     "\"mean_code\":%.6f," 
                     "\"block_ac_power_avg\":%.3f," 
                     "\"block_range_avg\":%.3f," 
                     "\"block_abs_diff_avg\":%.3f},",
                     MQTT_CLIENT_ID,
                     elapsed_seconds,
                     actual_spi_frequency_hz,
                     sample_rate_hz,
                     effective_sample_rate,
                     adc_statistics.completed_blocks,
                     adc_statistics.sample_count,
                     adc_statistics.spi_errors,
                     adc_statistics.invalid_samples,
                     adc_statistics.clipped_samples,
                     adc_statistics.clipped_blocks,
                     adc_statistics.minimum,
                     adc_statistics.maximum,
                     adc_average,
                     (double)adc_statistics.block_power_sum / block_count,
                     (double)adc_statistics.block_range_sum / block_count,
                     (double)adc_statistics.block_diff_sum / block_count))
    {
        return false;
    }

    if (!json_append(payload, payload_size, &used,
                     "\"fft\":{" 
                     "\"requested\":%" PRIu64 "," 
                     "\"queued\":%" PRIu64 "," 
                     "\"completed\":%" PRIu64 "," 
                     "\"dropped\":%" PRIu64 "," 
                     "\"errors\":%" PRIu64 "," 
                     "\"time_avg_us\":%.3f," 
                     "\"time_max_us\":%" PRIu32 "," 
                     "\"resolution_hz\":%.6f," 
                     "\"input_mean_code\":%.6f," 
                     "\"input_rms_fs\":%.9f," 
                     "\"dominant_avg_hz\":%.3f," 
                     "\"dominant_avg_dbfs_peak\":%.3f," 
                     "\"strongest_instantaneous_hz\":%.3f," 
                     "\"strongest_instantaneous_dbfs_peak\":%.3f," 
                     "\"centroid_hz_avg\":%.3f," 
                     "\"flatness_avg\":%.6f},",
                     snapshot->requested,
                     snapshot->queued,
                     snapshot->completed,
                     snapshot->dropped,
                     snapshot->errors,
                     (double)snapshot->total_time_us / completed,
                     snapshot->maximum_time_us,
                     resolution_hz,
                     snapshot->input_mean_sum / completed,
                     snapshot->input_rms_sum / completed,
                     dominant_bin * resolution_hz,
                     20.0 * log10(fmax(dominant_peak_amplitude, FFT_AMPLITUDE_EPSILON)),
                     snapshot->strongest_instantaneous_bin * resolution_hz,
                     20.0 * log10(fmax(strongest_peak_amplitude, FFT_AMPLITUDE_EPSILON)),
                     snapshot->centroid_hz_sum / completed,
                     snapshot->flatness_sum / completed))
    {
        return false;
    }

    /* ------------------------------------------------------------------
     * FFT primaire : un point par bin, sur toute la plage calibrée.
     * Les statistiques sont calculées en puissance linéaire puis converties
     * en dBFS pour la transmission.
     * ------------------------------------------------------------------ */
    if (!json_append(payload, payload_size, &used,
                     "\"spectrum\":{" 
                     "\"first_bin\":%" PRIu32 "," 
                     "\"last_bin\":%" PRIu32 "," 
                     "\"f_start_hz\":%.3f," 
                     "\"bin_hz\":%.6f," 
                     "\"mean_dbfs\":[",
                     analyzed_first_bin,
                     analyzed_last_bin,
                     analyzed_first_bin * resolution_hz,
                     resolution_hz))
    {
        return false;
    }

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        if (bin > analyzed_first_bin && !json_append(payload, payload_size, &used, ","))
            return false;

        const double mean_power = snapshot->bin_power_sum[bin] / completed;
        const double mean_rms = spectral_power_to_rms_fs(mean_power);
        if (!json_append(payload, payload_size, &used, "%.3f",
                         rms_fs_to_dbfs(mean_rms)))
            return false;
    }

    if (!json_append(payload, payload_size, &used, "],\"max_dbfs\":["))
        return false;

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        if (bin > analyzed_first_bin && !json_append(payload, payload_size, &used, ","))
            return false;

        const double max_rms =
            spectral_power_to_rms_fs(snapshot->bin_power_maximum[bin]);
        if (!json_append(payload, payload_size, &used, "%.3f",
                         rms_fs_to_dbfs(max_rms)))
            return false;
    }

    if (!json_append(payload, payload_size, &used, "],\"min_dbfs\":["))
        return false;

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        if (bin > analyzed_first_bin && !json_append(payload, payload_size, &used, ","))
            return false;

        const double min_rms =
            spectral_power_to_rms_fs(snapshot->bin_power_minimum[bin]);
        if (!json_append(payload, payload_size, &used, "%.3f",
                         rms_fs_to_dbfs(min_rms)))
            return false;
    }

    if (!json_append(payload, payload_size, &used, "],\"std_pct\":["))
        return false;

    for (uint32_t bin = analyzed_first_bin; bin <= analyzed_last_bin; bin++)
    {
        if (bin > analyzed_first_bin && !json_append(payload, payload_size, &used, ","))
            return false;

        const double mean_power = snapshot->bin_power_sum[bin] / completed;
        const double mean_square = snapshot->bin_power_squared_sum[bin] / completed;
        const double variance = fmax(mean_square - mean_power * mean_power, 0.0);
        const double std_pct =
            100.0 * sqrt(variance) / fmax(mean_power, FFT_POWER_EPSILON);

        if (!json_append(payload, payload_size, &used, "%.3f", std_pct))
            return false;
    }

    if (!json_append(payload, payload_size, &used, "]," 
                     "\"total_power_db\":%.3f," 
                     "\"dominant_share_pct\":%.3f," 
                     "\"correction_low_hz\":%.0f," 
                     "\"correction_high_hz\":%.0f},",
                     10.0 * log10(fmax(total_power, FFT_POWER_EPSILON)),
                     100.0 * dominant_power / fmax(total_power, FFT_POWER_EPSILON),
                     ANALYSIS_LOWER_HZ,
                     ANALYSIS_UPPER_HZ))
    {
        return false;
    }

    /* Bandes : uniquement des indicateurs dérivés, conservés pour le dashboard. */
    if (!json_append(payload, payload_size, &used, "\"bands\":["))
        return false;

    for (uint32_t band = 0U; band < FREQUENCY_BAND_COUNT; band++)
    {
        if (band > 0U && !json_append(payload, payload_size, &used, ","))
            return false;

        const double mean_power = snapshot->band_power_sum[band] / completed;
        const double mean_power_squared =
            snapshot->band_power_squared_sum[band] / completed;
        const double variance =
            fmax(mean_power_squared - mean_power * mean_power, 0.0);
        const double variability_percent =
            100.0 * sqrt(variance) / fmax(mean_power, FFT_POWER_EPSILON);

        if (!json_append(payload, payload_size, &used,
                         "{\"name\":\"%s\"," 
                         "\"low_hz\":%.0f," 
                         "\"high_hz\":%.0f," 
                         "\"dbfs_rms\":%.3f," 
                         "\"variability_pct\":%.3f}",
                         frequency_bands[band].name,
                         frequency_bands[band].lower_hz,
                         frequency_bands[band].upper_hz,
                         rms_fs_to_dbfs(spectral_power_to_rms_fs(mean_power)),
                         variability_percent))
        {
            return false;
        }
    }

    return json_append(payload, payload_size, &used,
                       "],\"correction\":{" 
                       "\"analysis_low_hz\":%.0f," 
                       "\"analysis_high_hz\":%.0f," 
                       "\"input_attenuation_ratio\":%.6f," 
                       "\"sensor_reference_output_vrms\":%.6f}}",
                       ANALYSIS_LOWER_HZ,
                       ANALYSIS_UPPER_HZ,
                       INPUT_ATTENUATION_RATIO,
                       SENSOR_REFERENCE_OUTPUT_VRMS);
}

static bool mqtt_publish_payload(const char *payload)
{
    if (!wifi_connect_for_publish())
    {
        return false;
    }

    char mqtt_uri[96];

    snprintf(
        mqtt_uri,
        sizeof(mqtt_uri),
        "mqtt://%s:%d",
        MQTT_BROKER,
        MQTT_PORT);

    const esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = mqtt_uri,
        .credentials.client_id = MQTT_CLIENT_ID,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASSWORD,
        .network.timeout_ms = 5000,
    };

    xEventGroupClearBits(
        mqtt_event_group,
        MQTT_CONNECTED_BIT | MQTT_PUBLISHED_BIT);

    esp_mqtt_client_handle_t client =
        esp_mqtt_client_init(&mqtt_config);

    if (client == NULL)
    {
        ESP_LOGE(TAG, "esp_mqtt_client_init a échoué");
        wifi_stop_after_publish();
        return false;
    }

    esp_err_t result = esp_mqtt_client_register_event(
        client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Enregistrement callback MQTT impossible: %s",
            esp_err_to_name(result));
        esp_mqtt_client_destroy(client);
        wifi_stop_after_publish();
        return false;
    }

    result = esp_mqtt_client_start(client);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Démarrage MQTT impossible: %s",
            esp_err_to_name(result));
        esp_mqtt_client_destroy(client);
        wifi_stop_after_publish();
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(
        mqtt_event_group,
        MQTT_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(MQTT_CONNECT_TIMEOUT_MS));

    bool success = false;

    if ((bits & MQTT_CONNECTED_BIT) != 0U)
    {
        xEventGroupClearBits(
            mqtt_event_group,
            MQTT_PUBLISHED_BIT);

        const int message_id = esp_mqtt_client_publish(
            client,
            MQTT_TOPIC,
            payload,
            0,
            MQTT_QOS,
            MQTT_RETAIN);

        if (message_id >= 0)
        {
            bits = xEventGroupWaitBits(
                mqtt_event_group,
                MQTT_PUBLISHED_BIT,
                pdTRUE,
                pdFALSE,
                pdMS_TO_TICKS(MQTT_PUBLISH_TIMEOUT_MS));

            success =
                (bits & MQTT_PUBLISHED_BIT) != 0U;
        }
    }

    if (success)
    {
        ESP_LOGI(
            TAG,
            "MQTT publié: topic=%s, %u octets",
            MQTT_TOPIC,
            (unsigned)strlen(payload));
    }
    else
    {
        ESP_LOGE(
            TAG,
            "Publication MQTT échouée ou expirée");
    }

    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    wifi_stop_after_publish();

    return success;
}

/* --------------------------------------------------------------------------
 * Rapport ADC + profil spectral
 * -------------------------------------------------------------------------- */

static void print_report(int64_t now_us)
{
    const int64_t elapsed_us = now_us - adc_statistics.start_time_us;

    if (elapsed_us <= 0 || adc_statistics.sample_count == 0U)
    {
        ESP_LOGW(TAG, "Aucun échantillon reçu");
        adc_statistics_reset(now_us);
        return;
    }

    wait_for_fft_jobs_to_finish();
    take_fft_statistics_snapshot();

    const double elapsed_seconds = (double)elapsed_us / 1000000.0;
    const double effective_sample_rate =
        (double)adc_statistics.sample_count / elapsed_seconds;
    const double block_sample_rate =
        (double)actual_spi_frequency_hz / (double)BITS_PER_SAMPLE;
    const double average =
        (double)adc_statistics.sum / (double)adc_statistics.sample_count;

    const double minimum_voltage =
        (double)adc_statistics.minimum * 3.3 / 4096.0;
    const double maximum_voltage =
        (double)adc_statistics.maximum * 3.3 / 4096.0;
    const double average_voltage = average * 3.3 / 4096.0;

    const double acquisition_efficiency =
        100.0 * effective_sample_rate / block_sample_rate;
    const double dead_time_percent = 100.0 - acquisition_efficiency;

    const double block_count =
        fmax((double)adc_statistics.completed_blocks, 1.0);

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(
        TAG,
        "Durée=%.6f s, SPI=%d Hz, bloc=%u samples",
        elapsed_seconds,
        actual_spi_frequency_hz,
        SAMPLES_PER_BLOCK);

    ESP_LOGI(
        TAG,
        "DMA: blocs=%" PRIu32 ", samples=%" PRIu64
        ", erreurs SPI=%" PRIu32,
        adc_statistics.completed_blocks,
        adc_statistics.sample_count,
        adc_statistics.spi_errors);

    ESP_LOGI(
        TAG,
        "Fs bloc=%.1f S/s, Fs moyen=%.1f S/s"
        ", efficacité=%.3f %%, temps mort=%.3f %%",
        block_sample_rate,
        effective_sample_rate,
        acquisition_efficiency,
        dead_time_percent);

    ESP_LOGI(
        TAG,
        "ADC codes: min=%" PRIu16 ", max=%" PRIu16
        ", moyenne=%.3f; tension=%.6f..%.6f V, moyenne=%.6f V",
        adc_statistics.minimum,
        adc_statistics.maximum,
        average,
        minimum_voltage,
        maximum_voltage,
        average_voltage);

    ESP_LOGI(
        TAG,
        "Blocs temporels: puissance AC moyenne=%.1f codes^2"
        " (%" PRIu64 "..%" PRIu64 ")"
        ", étendue moyenne=%.1f (%" PRIu32 "..%" PRIu32 ")"
        ", |diff| moyen=%.2f (%" PRIu32 "..%" PRIu32 ")",
        (double)adc_statistics.block_power_sum / block_count,
        adc_statistics.block_power_minimum,
        adc_statistics.block_power_maximum,
        (double)adc_statistics.block_range_sum / block_count,
        adc_statistics.block_range_minimum,
        adc_statistics.block_range_maximum,
        (double)adc_statistics.block_diff_sum / block_count,
        adc_statistics.block_diff_minimum,
        adc_statistics.block_diff_maximum);

    ESP_LOGI(
        TAG,
        "Erreurs cadrage=%" PRIu32 ", écrêtage=%" PRIu64
        " samples dans %" PRIu32 " blocs (%.3f %% des samples)",
        adc_statistics.invalid_samples,
        adc_statistics.clipped_samples,
        adc_statistics.clipped_blocks,
        100.0 * (double)adc_statistics.clipped_samples /
            fmax((double)adc_statistics.sample_count, 1.0));

    if (adc_statistics.clipped_samples > 0U)
    {
        ESP_LOGW(
            TAG,
            "ÉCRÊTAGE ADC: les puissances spectrales et la correction de transfert sont sous-estimées/déformées");
    }

    print_fft_report(block_sample_rate);

    static char mqtt_payload[MQTT_PAYLOAD_SIZE];

    if (build_mqtt_payload(
            mqtt_payload,
            sizeof(mqtt_payload),
            elapsed_us,
            block_sample_rate))
    {
        mqtt_publish_payload(mqtt_payload);
    }
    else
    {
        ESP_LOGE(
            TAG,
            "Construction du rapport JSON MQTT impossible");
    }

    /*
     * Important : le nouveau cycle commence APRES l'arrêt du Wi-Fi.
     * Le temps consacré à la connexion/publication n'entre donc pas dans
     * le calcul du Fs effectif de la période suivante.
     */
    adc_statistics_reset(esp_timer_get_time());
}

/* --------------------------------------------------------------------------
 * Initialisation SPI et DMA
 * -------------------------------------------------------------------------- */

static void initialize_spi(void)
{
    const spi_bus_config_t bus_config = {
        .mosi_io_num = ADS7049_PIN_FRAME,
        .miso_io_num = ADS7049_PIN_SDO,
        .sclk_io_num = ADS7049_PIN_SCLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = DMA_BLOCK_BYTES,
        .flags = SPICOMMON_BUSFLAG_MASTER |
                 SPICOMMON_BUSFLAG_MOSI |
                 SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_SCLK,
    };

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = ADS7049_SPI_CLOCK_REQUEST_HZ,
        .mode = 0,
        .spics_io_num = GPIO_NUM_NC,
        .queue_size = DMA_BUFFER_COUNT,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            ADS7049_SPI_HOST,
            &bus_config,
            SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(
        spi_bus_add_device(
            ADS7049_SPI_HOST,
            &device_config,
            &adc_spi));

    int actual_spi_frequency_khz = 0;
    ESP_ERROR_CHECK(
        spi_device_get_actual_freq(
            adc_spi,
            &actual_spi_frequency_khz));

    actual_spi_frequency_hz = actual_spi_frequency_khz * 1000;

    ESP_LOGI(
        TAG,
        "SPI demandé=%d Hz, SPI réel=%d Hz",
        ADS7049_SPI_CLOCK_REQUEST_HZ,
        actual_spi_frequency_hz);

    ESP_LOGI(
        TAG,
        "FRAME=D1/GPIO%d, SCLK=D8/GPIO%d, SDO=D9/GPIO%d",
        ADS7049_PIN_FRAME,
        ADS7049_PIN_SCLK,
        ADS7049_PIN_SDO);
}

static void initialize_dma_buffers(void)
{
    for (uint32_t index = 0; index < DMA_BUFFER_COUNT; index++)
    {
        dma_buffer_t *buffer = &dma_buffers[index];

        buffer->tx_buffer = heap_caps_malloc(
            DMA_BLOCK_BYTES,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

        buffer->rx_buffer = heap_caps_malloc(
            DMA_BLOCK_BYTES,
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

        if (buffer->tx_buffer == NULL || buffer->rx_buffer == NULL)
        {
            ESP_LOGE(
                TAG,
                "Allocation DMA impossible pour le tampon %" PRIu32,
                index);
            abort();
        }

        initialize_frame_pattern(buffer->tx_buffer);
        memset(buffer->rx_buffer, 0, DMA_BLOCK_BYTES);
        memset(&buffer->transaction, 0, sizeof(buffer->transaction));

        buffer->transaction.length = DMA_BLOCK_BYTES * 8U;
        buffer->transaction.rxlength = DMA_BLOCK_BYTES * 8U;
        buffer->transaction.tx_buffer = buffer->tx_buffer;
        buffer->transaction.rx_buffer = buffer->rx_buffer;
        buffer->transaction.user = buffer;
    }

    ESP_LOGI(
        TAG,
        "Double tampon DMA: %u × %u octets",
        DMA_BUFFER_COUNT,
        DMA_BLOCK_BYTES);
}

static void calibrate_ads7049(void)
{
    dma_buffer_t *buffer = &dma_buffers[0];

    ESP_LOGI(TAG, "Calibration initiale ADS7049");

    ESP_ERROR_CHECK(
        spi_device_polling_transmit(
            adc_spi,
            &buffer->transaction));

    esp_rom_delay_us(10);
    memset(buffer->rx_buffer, 0, DMA_BLOCK_BYTES);

    ESP_LOGI(TAG, "Calibration terminée");
}

/* --------------------------------------------------------------------------
 * Initialisation FFT
 * -------------------------------------------------------------------------- */

static void initialize_fft(void)
{
    fft_statistics_mutex = xSemaphoreCreateMutex();
    if (fft_statistics_mutex == NULL)
    {
        ESP_LOGE(TAG, "Création mutex FFT impossible");
        abort();
    }

    fft_job_queue = xQueueCreate(FFT_QUEUE_LENGTH, sizeof(fft_job_t));
    if (fft_job_queue == NULL)
    {
        ESP_LOGE(TAG, "Création file FFT impossible");
        abort();
    }

    const esp_err_t result = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Initialisation ESP-DSP impossible: %s",
            esp_err_to_name(result));
        abort();
    }

    dsps_wind_hann_f32(fft_window, FFT_SIZE);

    fft_window_sum = 0.0;
    fft_window_squared_sum = 0.0;

    for (uint32_t index = 0; index < FFT_SIZE; index++)
    {
        fft_window_sum += fft_window[index];
        fft_window_squared_sum +=
            (double)fft_window[index] * (double)fft_window[index];
    }

    xSemaphoreTake(fft_statistics_mutex, portMAX_DELAY);
    fft_statistics_reset_locked();
    xSemaphoreGive(fft_statistics_mutex);

    const BaseType_t task_result = xTaskCreatePinnedToCore(
        fft_task,
        "fft_task",
        FFT_TASK_STACK_SIZE,
        NULL,
        FFT_TASK_PRIORITY,
        NULL,
        FFT_TASK_CORE);

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Création tâche FFT impossible");
        abort();
    }

    ESP_LOGI(
        TAG,
        "FFT initialisée: N=%u, Hann, intervalle pseudo-aléatoire %u..%u blocs",
        FFT_SIZE,
        FFT_INTERVAL_MIN_BLOCKS,
        FFT_INTERVAL_MIN_BLOCKS + FFT_INTERVAL_SPAN_BLOCKS - 1U);
}

/* --------------------------------------------------------------------------
 * Tâche d'acquisition
 * -------------------------------------------------------------------------- */

static void acquisition_task(void *argument)
{
    (void)argument;

    for (uint32_t index = 0; index < DMA_BUFFER_COUNT; index++)
    {
        ESP_ERROR_CHECK(
            spi_device_queue_trans(
                adc_spi,
                &dma_buffers[index].transaction,
                portMAX_DELAY));
    }

    adc_statistics_reset(esp_timer_get_time());
    int64_t next_report_us =
        adc_statistics.start_time_us + REPORT_PERIOD_US;

    ESP_LOGI(
        TAG,
        "Acquisition démarrée: Fs bloc=%.1f S/s",
        (double)actual_spi_frequency_hz / (double)BITS_PER_SAMPLE);

    while (true)
    {
        spi_transaction_t *completed_transaction = NULL;

        esp_err_t result = spi_device_get_trans_result(
            adc_spi,
            &completed_transaction,
            portMAX_DELAY);

        if (result != ESP_OK)
        {
            adc_statistics.spi_errors++;
            ESP_LOGE(TAG, "Erreur DMA SPI: %s", esp_err_to_name(result));
            continue;
        }

        dma_buffer_t *completed_buffer =
            (dma_buffer_t *)completed_transaction->user;

        process_completed_block(completed_buffer);

        const int64_t now_us = esp_timer_get_time();

        if (now_us >= next_report_us)
        {
            /*
             * Ne pas remettre le tampon courant en file.
             * Il reste exactement DMA_BUFFER_COUNT-1 transaction(s) lancée(s).
             * On les draine sans analyser leurs échantillons afin que le SPI
             * soit complètement silencieux avant d'activer le Wi-Fi.
             */
            for (uint32_t pending = 1U;
                 pending < DMA_BUFFER_COUNT;
                 pending++)
            {
                spi_transaction_t *discarded_transaction = NULL;

                result = spi_device_get_trans_result(
                    adc_spi,
                    &discarded_transaction,
                    portMAX_DELAY);

                if (result != ESP_OK)
                {
                    adc_statistics.spi_errors++;
                    ESP_LOGE(
                        TAG,
                        "Drainage DMA avant MQTT impossible: %s",
                        esp_err_to_name(result));
                }
            }

            print_report(now_us);

            /*
             * La radio Wi-Fi est de nouveau arrêtée ici. On relance alors les
             * deux transactions DMA et seulement ensuite la période suivante.
             */
            for (uint32_t index = 0U;
                 index < DMA_BUFFER_COUNT;
                 index++)
            {
                result = spi_device_queue_trans(
                    adc_spi,
                    &dma_buffers[index].transaction,
                    portMAX_DELAY);

                if (result != ESP_OK)
                {
                    adc_statistics.spi_errors++;
                    ESP_LOGE(
                        TAG,
                        "Relance DMA après MQTT impossible: %s",
                        esp_err_to_name(result));
                }
            }

            next_report_us =
                adc_statistics.start_time_us + REPORT_PERIOD_US;
        }
        else
        {
            result = spi_device_queue_trans(
                adc_spi,
                &completed_buffer->transaction,
                portMAX_DELAY);

            if (result != ESP_OK)
            {
                adc_statistics.spi_errors++;
                ESP_LOGE(
                    TAG,
                    "Remise en file DMA impossible: %s",
                    esp_err_to_name(result));
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * Point d'entrée
 * -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "ADS7049 DMA + profil de bruit périodique + MQTT");
    ESP_LOGI(TAG, "Motif: 17 cycles CS bas + 7 cycles CS haut");
    ESP_LOGI(TAG, "MQTT topic: %s", MQTT_TOPIC);

    /*
     * Comme dans main_ref.c, réserver d'abord les ressources réseau pour
     * éviter une initialisation Wi-Fi tardive sur un heap déjà fragmenté.
     * La radio reste arrêtée après cette initialisation.
     */
    init_network_once();
    log_heap("après initialisation réseau");

    initialize_dma_buffers();
    initialize_spi();
    calibrate_ads7049();
    initialize_fft();

    const BaseType_t result = xTaskCreatePinnedToCore(
        acquisition_task,
        "ads7049_dma",
        ACQUISITION_TASK_STACK_SIZE,
        NULL,
        ACQUISITION_TASK_PRIORITY,
        NULL,
        ACQUISITION_TASK_CORE);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Création tâche acquisition impossible");
        abort();
    }
}
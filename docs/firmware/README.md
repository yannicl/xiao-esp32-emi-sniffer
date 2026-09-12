# Firmware Reference

This document describes the firmware implemented in [`main.c`](../../firmware/xiao-esp32s3/src/main.c). It is an implementation reference, not a promise of future behavior: values and interfaces below should be updated when the source changes.

## Scope

The firmware runs on a Seeed Studio XIAO ESP32-S3 and:

1. Samples an ADS7049 through the ESP32-S3 SPI2 peripheral.
2. Processes every completed 1024-sample block for ADC health metrics.
3. Sends selected blocks to a background FFT task.
4. Accumulates a ten-second spectral report.
5. Temporarily enables Wi-Fi, publishes one JSON report over MQTT, and stops the radio before acquisition resumes.

The firmware does not currently expose a command interface or store reports locally. Its runtime output is the ESP-IDF log stream and the MQTT report.

## Hardware interface

| Function | XIAO label | ESP32-S3 GPIO | Firmware detail |
| --- | --- | ---: | --- |
| ADC frame / CS pattern | D1 | GPIO2 | Driven as MOSI; the SPI hardware chip-select output is not used. |
| ADC clock | D8 | GPIO7 | SPI clock, mode 0. |
| ADC serial data | D9 | GPIO8 | MISO / ADS7049 SDO. |

The requested SPI clock is 24 MHz. The firmware reads back the actual configured frequency and derives the nominal sample rate as:

```text
sample_rate_hz = actual_spi_clock_hz / 24
```

Each conversion uses a 24-clock frame: 17 clocks with the frame signal low followed by 7 clocks with it high. The transmit buffer is initialized with the repeated MSB-first pattern `00 00 7F`. Received samples are decoded as 12-bit values from the first two received bytes; framing is marked invalid when the reserved bits do not match the expected zero pattern.

## Acquisition pipeline

- Two DMA buffers are allocated in internal DMA-capable memory.
- Each buffer contains 1024 samples, three bytes per sample, or 3072 bytes.
- The acquisition task runs on core 1 at priority 18 with a 6144-byte stack.
- DMA transactions are queued continuously during measurement.
- Every block contributes ADC statistics: minimum, maximum, mean, AC power, range, mean absolute sample-to-sample difference, invalid framing count, and clipping count.
- ADC codes at or below 4, or at or above 4091, are counted as clipped.
- A report is generated every 10 seconds. Pending DMA transactions are drained before Wi-Fi is enabled, preventing SPI acquisition and Wi-Fi activity from overlapping.

The report logs both the nominal block sample rate and the effective rate measured from the number of samples received during the reporting period. The time spent publishing is outside the following acquisition period.

## FFT processing

FFT processing runs asynchronously on core 0 at priority 10 with an 8192-byte stack. The FFT task receives jobs through a two-entry FreeRTOS queue.

- FFT size: 1024 samples.
- Window: Hann.
- FFT jobs: one block selected at a pseudo-random interval of 80 to 120 DMA blocks.
- Expected number of FFTs: approximately 100 per ten-second report, depending on the actual sample rate and queue availability.
- Analysis range: 40 kHz to 350 kHz.
- Frequency resolution: `sample_rate_hz / 1024`.
- The DC bin is excluded.
- The queue can drop a requested FFT when both entries are occupied; requested, queued, completed, dropped, and error counts are reported.

For each analyzed bin, the firmware accumulates mean, minimum, maximum, and relative standard deviation of corrected linear power. It also calculates:

- Dominant average frequency and peak level.
- Strongest instantaneous frequency and peak level.
- Spectral centroid.
- Spectral flatness.
- Total analyzed power.
- Four derived bands: 40-80 kHz, 80-150 kHz, 150-250 kHz, and 250-350 kHz.

## Transfer correction and units

The source contains a piecewise-linear sensor response curve from 40 kHz to 350 kHz. Spectral power is corrected against a reference sensor output of 0.255 Vrms and compensates for an additional input attenuation ratio of 0.5.

These values are an equalization based on the measured response in the source code. They are not an absolute mains-voltage calibration. Reported `dBFS` values and corrected power must not be interpreted as certified EMI-receiver levels or as a direct measurement of voltage on the AC circuit.

## Wi-Fi and MQTT behavior

Wi-Fi is initialized once at startup but remains stopped during acquisition. At the end of each report period it is started, connects as a WPA2 station, publishes one message, then disconnects and stops before the next acquisition cycle.

The MQTT topic is constructed as:

```text
emi/<MQTT_CLIENT_ID>/report
```

Current transport settings in `main.c`:

- MQTT QoS: 1.
- Retain flag: 0.
- Wi-Fi connection timeout: 10 seconds.
- MQTT connection timeout: 7 seconds.
- MQTT publish timeout: 7 seconds.
- MQTT URI scheme: `mqtt://`.
- MQTT network timeout: 5 seconds.

The current source configures MQTT with plain `mqtt://`; TLS certificates and secure `mqtts://` transport are not configured by this implementation. Use an appropriately protected network or extend the firmware before sending sensitive data over an untrusted network.

## MQTT report schema

Each report is a single JSON object with these top-level fields:

| Field | Contents |
| --- | --- |
| `client_id` | Configured MQTT client identifier. |
| `period_s` | Measurement period in seconds. |
| `spi_hz` | Actual SPI clock in hertz. |
| `sample_rate_hz` | Nominal rate derived from the SPI clock and 24-clock frame. |
| `effective_sample_rate_hz` | Samples received divided by elapsed acquisition time. |
| `adc` | Block, sample, error, clipping, code, power, range, and difference statistics. |
| `fft` | FFT job counters, timing, resolution, dominant frequency, centroid, and flatness. |
| `spectrum` | Frequency metadata plus `mean_dbfs`, `max_dbfs`, and `min_dbfs` arrays, one value per analyzed bin. |
| `bands` | Four derived band objects with limits, RMS level, and variability. |
| `correction` | Analysis limits, attenuation ratio, and sensor reference output. |

The `spectrum.mean_dbfs`, `max_dbfs`, and `min_dbfs` arrays correspond to bins from `first_bin` through `last_bin`, with spacing `bin_hz`. The first represented frequency is `f_start_hz`.

## Required local configuration

`main.c` includes `src/secrets.h`, but that file is not part of this repository. Create it locally and do not commit credentials. The source currently requires these macros:

```c
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASSWORD "your-wifi-password"
#define WIFI_TX_POWER_QDBM 40

#define MQTT_BROKER "192.0.2.10"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "emi-sniffer-01"
#define MQTT_USER "mqtt-user"
#define MQTT_PASSWORD "mqtt-password"

#define USE_STATIC_IP 0
#define WIFI_IP_ADDRESS "192.0.2.20"
#define WIFI_GATEWAY "192.0.2.1"
#define WIFI_SUBNET "255.255.255.0"
#define WIFI_DNS "192.0.2.1"
```

When `USE_STATIC_IP` is non-zero, all four IPv4 address macros are parsed and applied. Otherwise the firmware uses DHCP. The example values above are placeholders and must be replaced for a real network.

## Build and upload

The project is configured for PlatformIO with the `seeed_xiao_esp32s3` environment, Espressif32 platform 6.13.0, and the ESP-IDF framework:

```bash
cd firmware/xiao-esp32s3
pio run
pio run -t upload
pio device monitor -b 115200
```

The same source includes an ESP-IDF component registration file. A native ESP-IDF build may be used when the required ESP-IDF environment is installed, but PlatformIO is the documented project entry point. The firmware uses the ESP-DSP component and the ESP-IDF Wi-Fi, MQTT, SPI, timer, heap, and FreeRTOS APIs.

## Startup sequence

`app_main()` performs the following initialization:

1. Initialize NVS, networking objects, and Wi-Fi configuration; leave the radio stopped.
2. Allocate and initialize the two DMA buffers.
3. Initialize SPI2 and log the actual clock.
4. Perform one initial ADS7049 polling transfer.
5. Initialize the ESP-DSP FFT, Hann window, mutex, queue, and FFT task.
6. Start the acquisition task on core 1.

## Known limitations

- The firmware assumes the analog front end and ADS7049 input are correctly protected; it does not provide electrical isolation.
- The transfer correction is relative and depends on the measured sensor curve compiled into the source.
- ADC clipping distorts the spectral metrics; the firmware logs a warning but still publishes the report.
- MQTT publication failure is logged; there is no local retry queue or persistent storage.
- The JSON payload has a fixed 16384-byte buffer. If the generated spectrum does not fit, publication is skipped and an error is logged.
- `secrets.h` is required to compile but is intentionally not supplied here.

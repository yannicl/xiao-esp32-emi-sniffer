# XIAO ESP32-S3 Firmware

PlatformIO/ESP-IDF firmware for ADS7049 acquisition, FFT-based EMI analysis, and periodic MQTT reporting on the Seeed Studio XIAO ESP32-S3.

See the complete [firmware reference](../../docs/firmware/README.md) for:

- Hardware pins and ADS7049 SPI framing.
- DMA and FreeRTOS acquisition behavior.
- FFT parameters and transfer correction.
- MQTT report schema and networking behavior.
- Required local `secrets.h` configuration.
- Build, upload, and monitoring commands.

The implementation is in [`src/main.c`](src/main.c), and the PlatformIO environment is defined in [`platformio.ini`](platformio.ini).

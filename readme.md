# XIAO ESP32-S3 EMI Sniffer

An open-source residential EMI sniffer built around the Seeed Studio XIAO ESP32-S3. The project is intended to capture, analyze, and characterize high-frequency noise and interference present on 120 V AC power circuits.

The project is designed as an experimental platform for studying conducted EMI in residential electrical systems, with an emphasis on low-cost, reproducible hardware and open-source software.

> **Status: early development**
>
> The hardware, firmware, and measurement procedure are still being developed. Design files and results will be added as they are verified.

## Important safety notice

This project interfaces with potentially lethal mains voltage. A prototype must not be connected to a 120 V AC circuit unless the designer has verified the isolation, creepage, clearance, protection, enclosure, and measurement procedure. Do not touch, probe, or modify an energized circuit.

This project is for qualified experimenters and engineering evaluation. It is not a certified measurement instrument, electrical safety device, or substitute for an approved EMI receiver.

The hardware, firmware, schematics, PCB designs, and documentation are provided for experimental and educational purposes only. Use at your own risk. The authors provide no warranty and accept no responsibility for injury, death, property damage, electrical damage, fire, or any other loss resulting from the use, modification, construction, or operation of this project.

## Project goals

- Detect and characterize high-frequency interference on residential AC power circuits.
- Keep the design accessible to open-hardware builders and researchers.
- Make the analog signal path and its limitations measurable and reproducible.
- Stream or record sampled data for further analysis and visualization.

## System overview

The system combines a protected analog front end, high-speed ADC, ESP32-S3 data acquisition, FFT-based signal analysis, and MQTT connectivity for remote monitoring and data collection. Measurements can be analyzed over time to identify frequency bands, transient events, and characteristic signatures produced by household appliances and switching power supplies.

![EMI sniffer system overview](docs/system-overview.svg)

The diagram is also available as a standalone [system overview SVG](docs/system-overview.svg).


## Repository structure

### Hardware

- [AC front end](hardware/ac-front-end/): mains-side coupling, protection, schematic, PCB design, photos, and transfer-function measurements.
- [Signal-analysis circuit](hardware/signal-analysis/): conditioning and acquisition circuitry, schematic, PCB design, and assembly information.

### Firmware and software

- [XIAO ESP32-S3 firmware](firmware/xiao-esp32s3/): embedded acquisition, processing, and data transport code.
- [Visualization software](software/visualization/): tools for displaying and inspecting captured EMI data.

### Engineering records

- [Measurements](measurements/): calibration notes, transfer-function results, sample captures, and analysis data.
- [Documentation](docs/): build notes, operating procedure, design decisions, and supporting references.

## Current development plan

1. Define the AC-side coupling and protection strategy.
2. Capture the schematics and PCB designs in EasyEDA.
3. Characterize the analog path, including attenuation, bandwidth, and transfer function.
4. Implement acquisition firmware for the XIAO ESP32-S3.
5. Build a repeatable visualization and data-export workflow.
6. Publish verified measurements, limitations, and assembly notes.

## Reproducing the design

Until the first verified release is tagged, treat this repository as a development record. Before building anything:

1. Read the safety notice and review the latest design files.
2. Check that the documented ratings and protection measures match the intended installation.
3. Validate the signal path with a suitable isolated test setup before connecting to mains.
4. Record hardware revisions, firmware revisions, and measurement equipment with every result.

Build instructions, bill of materials, EasyEDA source files, firmware setup, and visualization installation instructions will be added to their respective directories as the design matures.

## Measurements

Measurements will document the test setup, equipment, circuit revision, firmware revision, stimulus, and processing method. Planned results include:

- AC-front-end transfer function.
- Noise floor and usable dynamic range.
- Frequency response and acquisition limits.
- Representative residential interference captures.

See [measurements](measurements/) for the working record.

## Contributions

Issues, test results, design review, and improvements to the documentation are welcome. When reporting a result, include the hardware revision, firmware revision, wiring or probe setup, and raw data when possible.

## Licensing

This repository uses separate licenses for hardware and software:

- **Hardware and hardware design files:** [CERN-OHL-P-2.0](LICENSES/CERN-OHL-P-2.0.txt). This applies to schematics, PCB layouts, bills of materials, fabrication files, and other hardware design source unless a file states otherwise.
- **Software:** [MIT](LICENSES/MIT.txt). This applies to firmware, visualization tools, scripts, and other software source unless a file states otherwise.

When redistributing a hardware design or product, retain the applicable notices and provide access to the CERN-OHL-P-2.0 license. When redistributing software, retain the MIT copyright and permission notice.

## Disclaimer

The author provides this project and its documentation without a guarantee that it is safe, accurate, compliant, or suitable for any particular use. Anyone building or using it is responsible for electrical safety, regulatory compliance, and independent validation.
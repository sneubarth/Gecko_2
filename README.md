# Gecko 2 LORA Projects

This repository contains test code for the RFM95 LoRa radio modules (915 MHz band, based on the Semtech SX1276 chip) integrated into the Gecko2 PCBs designed by Stuart Neubarth. The project consists of:

1. **Gecko_2_LORA_tx**: Transmitter project that collects sensor data (temperature, humidity, light level, battery voltage) and transmits it via LoRa radio every 2 seconds.
2. **Gecko_2_LORA_rx**: Receiver project that displays the transmitted data on an OLED screen and provides signal quality metrics.

Both projects are configured for long-range communication using optimized LoRa parameters (SF12, 62.5kHz bandwidth, 4/8 coding rate) and utilize shared libraries for hardware components.

## Project Structure

```
Gecko_2/
├── Gecko_2_LORA_tx/     # Transmitter project
│   ├── lib/             # Project-specific libraries
│   ├── platformio.ini   # PlatformIO configuration
│   └── src/             # Source code
│
├── Gecko_2_LORA_rx/     # Receiver project
│   ├── lib/             # Project-specific libraries
│   ├── platformio.ini   # PlatformIO configuration
│   └── src/             # Source code
│
└── shared_libs/         # Shared libraries
    ├── Gecko_RF95/      # LORA RF95 library
    ├── LED_RGB/         # RGB LED library
    └── LTC4162/         # Battery management library
```

## Setup and Usage

1. Install PlatformIO (if not already installed)
2. Open either project directory in PlatformIO
3. Build and upload to your device

## Dependencies

- PlatformIO
- Libraries in the shared_libs directory

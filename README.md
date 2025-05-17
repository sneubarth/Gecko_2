# Gecko 2 LORA Projects

This repository contains two PlatformIO projects for LORA communication:

1. **Gecko_2_LORA_tx**: Transmitter project
2. **Gecko_2_LORA_rx**: Receiver project

Both projects use shared libraries located in the `shared_libs` directory.

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

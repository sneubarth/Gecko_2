/**
 * @file Gecko_RF95.h
 * @brief Common definitions for RFM95 LoRa radio configuration
 * 
 * This file contains the frequency and modulation parameter definitions
 * for the RFM95 LoRa radio modules used in the Gecko project.
 */

#ifndef GECKO_RF95_H
#define GECKO_RF95_H

// Pin definitions for RFM95
#define RFM95_MISO 2    // MISO pin
#define RFM95_MOSI 42   // MOSI pin
#define RFM95_SCK 6     // SCK pin
#define RFM95_CS 47     // NSS (CS) pin
#define RFM95_RST 45    // RST pin
#define RFM95_INT 7     // Interrupt pin (connected to DIO0)

// Frequency setting - 918.75 MHz (within US 902-928 MHz ISM band)
#define RF95_FREQ 918.75

// LoRa LongFast modulation parameters (not used - kept for reference)
#define LORA_LF_SPREADING_FACTOR 10       // SF10 - good balance of range and speed
#define LORA_LF_SIGNAL_BANDWIDTH 125000   // 125 kHz - standard bandwidth
#define LORA_LF_CODING_RATE 5             // 4/5 coding rate - standard redundancy
#define LORA_LF_PREAMBLE_LENGTH 8         // Standard preamble length
#define LORA_LF_TX_POWER 23               // Maximum transmit power (23 dBm)
#define LORA_LF_TX_TIMEOUT 2000           // 2 seconds timeout for packet transmission

// LoRa VeryLongSlow modulation parameters
#define LORA_SPREADING_FACTOR 12          // SF12 - slowest data rate, longest range
#define LORA_SIGNAL_BANDWIDTH 62500       // 62.5 kHz - narrow bandwidth for better sensitivity
#define LORA_CODING_RATE 8                // 4/8 coding rate - highest redundancy
#define LORA_PREAMBLE_LENGTH 16           // Longer preamble for better reception
#define LORA_TX_POWER 23                  // Maximum transmit power (23 dBm)
#define LORA_TX_TIMEOUT 8000              // 8 seconds timeout for packet transmission (calculated for ~60 byte payload)

// Timing variables for transmitter
#define TX_INTERVAL 2000                  // 2 seconds between transmissions

// RSSI threshold for receiver
#define RSSI_THRESHOLD -50                // RSSI threshold for warning

#endif // GECKO_RF95_H

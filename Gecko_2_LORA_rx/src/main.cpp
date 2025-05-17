#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_VEML7700.h>
#include <RH_RF95.h>
#include "LED_RGB.h"
#include "LTC4162.h"
#include "Gecko_RF95.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// UART assignments
#define NANO_UART 1
#define MPPT_UART 2

// serial port assignments
#define MPPT_IN_PIN 18
#define MPPT_OUT_PIN 17

// #define NANO_IN_PIN 36
#define NANO_IN_PIN 39  // pin 32
#define NANO_OUT_PIN 38  // pin 31

// GPS uses software UART library
#define GPS_IN_PIN 4
#define GPS_OUT_PIN 5

// 5V nano regulator enable pin
#define PI_5V_EN_N_PIN 48 // now 5V enable
#define VBUS_5V_EN_PIN 46  // now USB VBUS
#define MDM_3V3_EN_N 14     // now 3V3 cell modem enable
#define SOFT_ON_SWITCH 3

// Removed FAN_1_PIN as it conflicts with RFM95_CS (pin 47)
#define FAN_2_PIN 21

#define BOOST_EN 10 // pin 18
// #define PWR_OUT_SW_EN 7 // pin 7  RE-PURPOSED Radio interrupt

// Constants for LED brightness scaling
#define MIN_LUX 1.0
#define MAX_LUX 1000.0
#define MIN_BRIGHTNESS 1
#define MAX_BRIGHTNESS 255

// RFM95 pin definitions are now in Gecko_RF95.h

// I2C pins
#define SDA_PIN 8  // I2C SDA
#define SCL_PIN 9  // I2C SCL

// OLED definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// LoRa parameters are now in Gecko_RF95.h
// RSSI threshold for signal quality estimation
#define RSSI_THRESHOLD -50

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Sensors
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();
LTC4162 battery;

// Flags to track sensor availability
bool veml7700_available = false;
bool oled_available = false;
bool ltc4162_available = false;
bool rfm95_available = false;

// Packet statistics
uint32_t packetsReceived = 0;
uint32_t failedReceives = 0;
uint32_t packetsLost = 0;
float averageRssi = 0.0;
uint32_t rssiCount = 0;
unsigned long lastPacketTime = 0;
int16_t lastPacketNum = -1;

// Stack monitoring
UBaseType_t getMinFreeStack() {
    return uxTaskGetStackHighWaterMark(NULL);
}

void printStackInfo(const char* label) {
    UBaseType_t stackLeft = getMinFreeStack();
    Serial.print("["); Serial.print(millis()); 
    Serial.print(" ms] Stack - "); Serial.print(label);
    Serial.print(": "); Serial.print(stackLeft);
    Serial.println(" words free");
}

float estimateSignalQuality(int16_t rssi) {
    if (rssi <= -100) return 0.0;
    if (rssi >= -20) return 100.0;
    return (float)(rssi + 100) * 100.0 / 80.0;
}

bool isValidPayload(const char *payload) {
    if (strlen(payload) < 20) return false;
    if (strstr(payload, "#") != payload) return false;
    return strstr(payload, " T:") && strstr(payload, " H:") &&
           strstr(payload, " TO:") && strstr(payload, " HO:") &&
           strstr(payload, " LUX:") && strstr(payload, " BAT:");
}

void hw_pins_init(void) {
    pinMode(MDM_3V3_EN_N, OUTPUT);
    digitalWrite(MDM_3V3_EN_N, HIGH); // modem off

    pinMode(PI_5V_EN_N_PIN, OUTPUT);
    digitalWrite(PI_5V_EN_N_PIN, HIGH); // Pi off

    pinMode(VBUS_5V_EN_PIN, OUTPUT);
    digitalWrite(VBUS_5V_EN_PIN, LOW);  // USB power off to CAM

    pinMode(SOFT_ON_SWITCH, INPUT_PULLUP);

    pinMode(MPPT_OUT_PIN, OUTPUT);
    digitalWrite(MPPT_OUT_PIN, LOW);    // force low so don't drive when no 5V = U6 and save power

    pinMode(BOOST_EN, OUTPUT);
    digitalWrite(BOOST_EN, LOW);
}

void setup() {
    // Initialize watchdog timer with a timeout of 30 seconds
    esp_task_wdt_init(30, true);
    // Subscribe this task to the watchdog timer
    esp_task_wdt_add(NULL);
    
    hw_pins_init();

    delay(3000);
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    
    // Print initial stack info
    printStackInfo("Setup Start");

    // Initialize I2C
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing I2C on SDA: GPIO8, SCL: GPIO9...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // Initialize LED
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing LED...");
    led_rgb_init();
    
    // Initialize VEML7700 to check light level for LED brightness
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing VEML7700...");
    if (!veml7700.begin()) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] VEML7700 init failed, continuing without light sensor");
        veml7700_available = false;
    } else {
        veml7700_available = true;
        veml7700.setGain(VEML7700_GAIN_1);
        veml7700.setIntegrationTime(VEML7700_IT_100MS);
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] VEML7700 init OK!");
    }
    
    // Set LED brightness - use default if light sensor not available
    uint8_t led_brightness = MAX_BRIGHTNESS / 2; // Default to mid brightness
    float init_lux = 0.0;
    
    if (veml7700_available) {
        // Scale LED brightness based on ambient light with a linear curve
        init_lux = veml7700.readLux();
        if (init_lux >= MAX_LUX) {
            led_brightness = MAX_BRIGHTNESS; // Full brightness for outdoor/bright environments
        } else if (init_lux <= MIN_LUX) {
            led_brightness = MIN_BRIGHTNESS; // Minimum brightness for very dark environments
        } else {
            // Linear scaling between MIN_BRIGHTNESS and MAX_BRIGHTNESS for lux values between MIN_LUX and MAX_LUX
            led_brightness = MIN_BRIGHTNESS + (init_lux - MIN_LUX) * (MAX_BRIGHTNESS - MIN_BRIGHTNESS) / (MAX_LUX - MIN_LUX);
        }
    }
    
    led_rgb_blink(255, 0, 0, led_brightness, 500); // Red, 500ms
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] LED initialized (");
    if (veml7700_available) {
        Serial.print("Lux: "); Serial.print(init_lux, 1); Serial.print(", ");
    }
    Serial.print("Brightness: "); Serial.print(led_brightness); Serial.println(")");

    // Initialize OLED
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing SSD1306...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] SSD1306 init failed, continuing without display");
        oled_available = false;
    } else {
        oled_available = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("SSD1306 init OK");
        display.display();
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] SSD1306 init OK!");
    }

    // Initialize LTC4162
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing LTC4162...");
    if (!battery.begin()) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 init failed, continuing with default voltage");
        ltc4162_available = false;
    } else {
        ltc4162_available = true;
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 init OK!");
    }

    // Initialize SPI
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing SPI...");
    SPI.begin(RFM95_SCK, RFM95_MISO, RFM95_MOSI);
    
    // Check stack before RFM95 initialization
    printStackInfo("Before RFM95 Init");
    
    // Feed the watchdog timer
    esp_task_wdt_reset();

    // Initialize RFM95
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing RFM95...");
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    
    if (!rf95.init()) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] RFM95 init failed, cannot continue without radio");
        rfm95_available = false;
        // Flash LED to indicate error
        for (int i = 0; i < 10; i++) {
            led_rgb_blink(255, 0, 0, MAX_BRIGHTNESS, 200); // Red, 200ms
            delay(200);
        }
        Serial.println("System will continue but radio functionality will be disabled");
    } else if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] setFrequency failed, cannot continue without radio");
        rfm95_available = false;
        // Flash LED to indicate error
        for (int i = 0; i < 10; i++) {
            led_rgb_blink(255, 0, 0, MAX_BRIGHTNESS, 200); // Red, 200ms
            delay(200);
        }
        Serial.println("System will continue but radio functionality will be disabled");
    } else {
        rfm95_available = true;
        
        // Configure for LoRa mode with parameters optimized for maximum range
        /*
         * LoRa Modulation Parameters for Range vs. Data Rate:
         * 
         * Spreading Factor (SF): Higher SF = Longer Range, Slower Data Rate
         *   - SF7  : Fastest data rate, shortest range
         *   - SF8  : |
         *   - SF9  : |
         *   - SF10 : |
         *   - SF11 : |
         *   - SF12 : Slowest data rate, longest range
         * 
         * Bandwidth (BW): Lower BW = Longer Range, Slower Data Rate
         *   - 500 kHz  : Fastest data rate, shortest range
         *   - 250 kHz  : |
         *   - 125 kHz  : | (standard)
         *   - 62.5 kHz : |
         *   - 41.7 kHz : |
         *   - 31.25 kHz: |
         *   - 20.8 kHz : |
         *   - 15.6 kHz : Slowest data rate, longest range
         * 
         * Coding Rate (CR): Higher CR = Better Error Correction, Slower Data Rate
         *   - 4/5: Least redundancy, fastest data rate
         *   - 4/6: |
         *   - 4/7: |
         *   - 4/8: Most redundancy, best error correction
         * 
         * Transmit Power: Higher Power = Longer Range
         *   - 5 dBm to 23 dBm (max for RFM95)
         */
        
        // Set transmitter power
        rf95.setTxPower(LORA_TX_POWER, false);
        
        // Configure for LoRa mode with LongFast parameters
        rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
        rf95.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
        rf95.setCodingRate4(LORA_CODING_RATE);
        rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);
        
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] RFM95 LoRa Receiver init OK!");
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] Frequency: "); Serial.print(RF95_FREQ); Serial.println(" MHz");
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LoRa configured with VeryLongSlow settings:");
        Serial.print("   - Spreading Factor: "); Serial.println(LORA_SPREADING_FACTOR);
        Serial.print("   - Bandwidth: "); Serial.print(LORA_SIGNAL_BANDWIDTH / 1000.0); Serial.println(" kHz");
        Serial.print("   - Coding Rate: 4/"); Serial.println(LORA_CODING_RATE);
        Serial.print("   - Preamble Length: "); Serial.println(LORA_PREAMBLE_LENGTH);
        
        // Check stack after RFM95 initialization
        printStackInfo("After RFM95 Init");
    }
    
    // Feed the watchdog timer at the end of setup
    esp_task_wdt_reset();
    printStackInfo("End of Setup");
}

void loop() {
    // Feed the watchdog timer periodically
    static unsigned long last_wdt_feed = 0;
    if (millis() - last_wdt_feed >= 5000) {
        esp_task_wdt_reset();
        last_wdt_feed = millis();
        
        // Print stack info every 10 seconds
        static unsigned long last_stack_print = 0;
        if (millis() - last_stack_print >= 10000) {
            printStackInfo("Loop Runtime");
            last_stack_print = millis();
        }
    }
    
    // Read local sensors periodically
    static unsigned long last_sensor_read = 0;
    if (veml7700_available && millis() - last_sensor_read >= 1000) {
        // Read local light level for debugging
        float local_lux = veml7700.readLux();
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] Local Lux: ");
        Serial.println(local_lux, 1);
        last_sensor_read = millis();
    }

    // Read receiver battery voltage (every 1s to reduce logging)
    static unsigned long last_voltage_read = 0;
    static float last_rx_voltage = -1.0;
    float rx_battery_voltage = 0.0;
    bool valid_voltage = false;
    if (millis() - last_voltage_read >= 1000) {
        ltc4162_status_t rx_bat_status = battery.readBatteryVoltage(&rx_battery_voltage);
        valid_voltage = (rx_bat_status == LTC4162_OK);
        last_voltage_read = millis();
    } else {
        rx_battery_voltage = last_rx_voltage;
        valid_voltage = true; // Assume cached voltage is valid
    }

    // Log telemetry issue only once
    static bool error_logged = false;
    if (!valid_voltage && !error_logged) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162: Unable to read battery voltage");
        error_logged = true;
    } else if (valid_voltage && error_logged) {
        error_logged = false;
    }

    // Log voltage only on change
    if (valid_voltage && abs(rx_battery_voltage - last_rx_voltage) > 0.1) {
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] RX Battery Voltage: ");
        Serial.print(rx_battery_voltage, 1); Serial.println(" V");
        last_rx_voltage = rx_battery_voltage;
    }

    if (rfm95_available && rf95.available()) {
        // Check stack before packet reception
        printStackInfo("Before Packet Reception");
        
        // Feed the watchdog timer before packet reception
        esp_task_wdt_reset();
        
        uint8_t buf[60];
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
            int16_t rssi = rf95.lastRssi();
            buf[len] = 0;
            unsigned long currentTime = millis();

            // Validate payload
            char *payload = (char *)buf;
            if (!isValidPayload(payload)) {
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Invalid Payload: ");
                Serial.println(payload);
                failedReceives++;
                if (oled_available) {
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(SSD1306_WHITE);
                    display.setCursor(0, 0); display.print("RX #----");
                    display.setCursor(0, 10); display.print("TI: ---.-  TO: ---.-");
                    display.setCursor(0, 20); display.print("HI: --  HO: --");
                    display.setCursor(0, 30); display.print("LUX: ----.-");
                    display.setCursor(0, 40); 
                    display.print("B TX: --.- RX: ");
                    if (valid_voltage) {
                        display.print(rx_battery_voltage, 1); display.print(" V");
                    } else {
                        display.print("--.- V");
                    }
                    display.setCursor(0, 50); display.print("RSSI: "); display.print(rssi); display.print(" dBm");
                    display.display();
                }
                return;
            }

            // Parse packet
            float tempF_inside, tempF_outside, lux, tx_battery_voltage;
            int humidity_inside, humidity_outside, packetNum;
            int fields = sscanf(payload, "#%d T:%f H:%d TO:%f HO:%d LUX:%f BAT:%f",
                                &packetNum, &tempF_inside, &humidity_inside, &tempF_outside,
                                &humidity_outside, &lux, &tx_battery_voltage);
            if (fields != 7) {
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Parse Error (");
                Serial.print(fields); Serial.print(" fields): "); Serial.println(payload);
                failedReceives++;
                if (oled_available) {
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(SSD1306_WHITE);
                    display.setCursor(0, 0); display.print("RX #----");
                    display.setCursor(0, 10); display.print("TI: ---.-  TO: ---.-");
                    display.setCursor(0, 20); display.print("HI: --  HO: --");
                    display.setCursor(0, 30); display.print("LUX: ----.-");
                    display.setCursor(0, 40); 
                    display.print("B TX: --.- RX: ");
                    if (valid_voltage) {
                        display.print(rx_battery_voltage, 1); display.print(" V");
                    } else {
                        display.print("--.- V");
                    }
                    display.setCursor(0, 50); display.print("RSSI: "); display.print(rssi); display.print(" dBm");
                    display.display();
                }
                return;
            }

            // Packet loss detection
            if (lastPacketNum >= 0 && packetNum > lastPacketNum + 1) {
                packetsLost += packetNum - lastPacketNum - 1;
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Packet Loss Detected: ");
                Serial.print(packetsLost); Serial.println(" packets lost");
            }
            lastPacketNum = packetNum;
            packetsReceived++;
            averageRssi += rssi;
            rssiCount++;

            // Scale LED brightness based on ambient light with a linear curve
            uint8_t led_brightness = MAX_BRIGHTNESS / 2; // Default to mid brightness
            
            if (veml7700_available) {
                float current_lux = veml7700.readLux();
                Serial.print("["); Serial.print(millis()); Serial.print(" ms] Current Lux: ");
                Serial.println(current_lux, 1);
                
                if (current_lux >= MAX_LUX) {
                    led_brightness = MAX_BRIGHTNESS; // Full brightness for outdoor/bright environments
                } else if (current_lux <= MIN_LUX) {
                    led_brightness = MIN_BRIGHTNESS; // Minimum brightness for very dark environments
                } else {
                    // Linear scaling between MIN_BRIGHTNESS and MAX_BRIGHTNESS for lux values between MIN_LUX and MAX_LUX
                    led_brightness = MIN_BRIGHTNESS + (current_lux - MIN_LUX) * (MAX_BRIGHTNESS - MIN_BRIGHTNESS) / (MAX_LUX - MIN_LUX);
                }
            }
            led_rgb_blink(255, 0, 0, led_brightness, 100); // Red, 100ms
            
            Serial.print("["); Serial.print(millis()); Serial.print(" ms] LED Brightness: ");
            Serial.print(led_brightness);
            if (veml7700_available) {
                Serial.print(" (Lux: "); Serial.print(veml7700.readLux(), 1); Serial.println(")");
            } else {
                Serial.println(" (Lux sensor not available)");
            }

            // Update OLED with increased spacing between lines
            if (oled_available) {
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0); display.print("RX #"); display.printf("%04d", packetNum);
                display.setCursor(0, 10); display.print("TI: "); display.print(tempF_inside, 1);
                display.print("  TO: "); display.print(tempF_outside, 1);
                display.setCursor(0, 20); display.print("HI: "); display.print(humidity_inside);
                display.print("  HO: "); display.print(humidity_outside);
                display.setCursor(0, 30); display.print("LUX: "); display.print(lux, 1);
                display.setCursor(0, 40); 
                display.print("B TX: "); display.print(tx_battery_voltage, 1);
                display.print(" RX: "); 
                if (valid_voltage) {
                    display.print(rx_battery_voltage, 1); display.print(" V");
                } else {
                    display.print("--.- V");
                }
                display.setCursor(0, 50); display.print("RSSI: "); display.print(rssi); display.print(" dBm");
                display.display();
            }

            // Serial output
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Received Payload: ");
            Serial.println(payload);
            if (valid_voltage && abs(rx_battery_voltage - last_rx_voltage) > 0.1) {
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] RX Battery Voltage: ");
                Serial.print(rx_battery_voltage, 1); Serial.println(" V");
                last_rx_voltage = rx_battery_voltage;
            }
            if (lastPacketTime > 0) {
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Packet Interval: ");
                Serial.print(currentTime - lastPacketTime); Serial.println(" ms");
            }
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] RSSI: ");
            Serial.print(rssi); Serial.println(" dBm");
            if (rssi < RSSI_THRESHOLD) {
                Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Warning: Low RSSI (below ");
                Serial.print(RSSI_THRESHOLD); Serial.println(" dBm)");
            }
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Signal Quality: ");
            Serial.print(estimateSignalQuality(rssi)); Serial.println("%");
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Packets Received: ");
            Serial.println(packetsReceived);
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Packets Lost: ");
            Serial.println(packetsLost);
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Failed Receives: ");
            Serial.println(failedReceives);
            Serial.print("["); Serial.print(currentTime); Serial.print(" ms] Average RSSI: ");
            Serial.print(rssiCount > 0 ? averageRssi / rssiCount : 0.0); Serial.println(" dBm");
            lastPacketTime = currentTime;
            
            // Check stack after packet processing
            printStackInfo("After Packet Processing");
        } else {
            failedReceives++;
            Serial.print("["); Serial.print(millis()); Serial.println(" ms] Receive failed");
        }
    }
}

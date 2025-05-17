#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_SHT31.h>
#include <RH_RF95.h>
#include "LED_RGB.h"
#include "LTC4162.h"
#include "Gecko_RF95.h"

// I2C pins
#define SDA_PIN 8  // I2C SDA
#define SCL_PIN 9  // I2C SCL

// OLED definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Sensors
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();
Adafruit_SHT31 sht31 = Adafruit_SHT31();
LTC4162 battery;

// Flags to track sensor availability
bool veml7700_available = false;
bool oled_available = false;
bool sht31_available = false;
bool ltc4162_available = false;
bool rfm95_available = false;

// Packet statistics
uint32_t packetsSent = 0;
unsigned long lastPacketTime = 0;

// Simulated outdoor sensor values (for demonstration)
float tempF_outside = 75.0;
int humidity_outside = 50;

// Constants for LED brightness scaling
#define MIN_LUX 1.0
#define MAX_LUX 1000.0
#define MIN_BRIGHTNESS 1
#define MAX_BRIGHTNESS 128  // Reduced to 1/2 of original (255/2)

// Function to convert Celsius to Fahrenheit
float celsiusToFahrenheit(float celsius) {
  return celsius * 9.0 / 5.0 + 32.0;
}

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

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("\n\n=== RFM95 LoRa Transmitter ===");
  
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
  
  // Initialize SHT31 temperature/humidity sensor
  Serial.print("["); Serial.print(millis()); Serial.println(" ms] Initializing SHT31...");
  if (!sht31.begin(0x44)) {
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] SHT31 init failed, continuing with simulated temperature/humidity");
    sht31_available = false;
  } else {
    sht31_available = true;
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] SHT31 init OK!");
  }
  
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
      led_rgb_blink(255, 0, 0, 255, 200); // Red, 200ms
      delay(200);
    }
    while (1); // Cannot continue without radio
  } else if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] setFrequency failed, cannot continue without radio");
    rfm95_available = false;
    // Flash LED to indicate error
    for (int i = 0; i < 10; i++) {
      led_rgb_blink(255, 0, 0, 255, 200); // Red, 200ms
      delay(200);
    }
    while (1); // Cannot continue without radio
  } else {
    rfm95_available = true;
    
    // Configure for LoRa mode with parameters matching the RX code
    rf95.setTxPower(LORA_TX_POWER, false);
    rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
    rf95.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
    rf95.setCodingRate4(LORA_CODING_RATE);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);
    
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] RFM95 LoRa Transmitter init OK!");
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] Frequency: "); Serial.print(RF95_FREQ); Serial.println(" MHz");
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] LoRa configured with the following settings:");
    Serial.print("   - Spreading Factor: "); Serial.println(LORA_SPREADING_FACTOR);
    Serial.print("   - Bandwidth: "); Serial.print(LORA_SIGNAL_BANDWIDTH / 1000.0); Serial.println(" kHz");
    Serial.print("   - Coding Rate: 4/"); Serial.println(LORA_CODING_RATE);
    Serial.print("   - Preamble Length: "); Serial.println(LORA_PREAMBLE_LENGTH);
    
    // Check stack after RFM95 initialization
    printStackInfo("After RFM95 Init");
  }
  
  // Indicate setup complete with green LED
  led_rgb_blink(0, 255, 0, 128, 1000); // Green, 1 second
  
  printStackInfo("End of Setup");
}

void loop() {
  // Print stack info every 10 seconds
  static unsigned long last_stack_print = 0;
  if (millis() - last_stack_print >= 10000) {
    printStackInfo("Loop Runtime");
    last_stack_print = millis();
  }
  
  // Send a packet every TX_INTERVAL milliseconds
  if (millis() - lastPacketTime >= TX_INTERVAL) {
    // Read sensor values
    float tempC_inside = 25.0;  // Default value
    float tempF_inside = 77.0;  // Default value
    int humidity_inside = 40;   // Default value
    float lux = 100.0;          // Default value
    float battery_voltage = 3.7; // Default value
    
    // Read temperature and humidity if sensor is available
    if (sht31_available) {
      tempC_inside = sht31.readTemperature();
      tempF_inside = celsiusToFahrenheit(tempC_inside);
      humidity_inside = (int)sht31.readHumidity();
      
      // Simulate outdoor values based on indoor values
      tempF_outside = tempF_inside - 5.0 + random(-30, 30) / 10.0; // Slightly cooler outside with some randomness
      humidity_outside = humidity_inside + 10 + random(-5, 15);    // Slightly more humid outside with some randomness
      
      // Ensure humidity is within valid range
      if (humidity_outside > 100) humidity_outside = 100;
      if (humidity_outside < 0) humidity_outside = 0;
    }
    
    // Read light level if sensor is available
    if (veml7700_available) {
      lux = veml7700.readLux();
    }
    
    // Read battery voltage if available
    if (ltc4162_available) {
      ltc4162_status_t bat_status = battery.readBatteryVoltage(&battery_voltage);
      if (bat_status != LTC4162_OK) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162: Unable to read battery voltage");
      }
    }
    
    // Format packet in the expected format for the RX code
    char packet[60];
    snprintf(packet, sizeof(packet), "#%lu T:%.1f H:%d TO:%.1f HO:%d LUX:%.1f BAT:%.1f", 
             packetsSent + 1, tempF_inside, humidity_inside, tempF_outside, humidity_outside, lux, battery_voltage);
    
    // Update OLED display
    if (oled_available) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0); display.print("TX #"); display.printf("%04lu", packetsSent + 1);
      display.setCursor(0, 10); display.print("TI: "); display.print(tempF_inside, 1);
      display.print("  TO: "); display.print(tempF_outside, 1);
      display.setCursor(0, 20); display.print("HI: "); display.print(humidity_inside);
      display.print("  HO: "); display.print(humidity_outside);
      display.setCursor(0, 30); display.print("LUX: "); display.print(lux, 1);
      display.setCursor(0, 40); display.print("BAT: "); display.print(battery_voltage, 1); display.print(" V");
      display.setCursor(0, 50); display.print("Packets Sent: "); display.print(packetsSent);
      display.display();
    }
    
    // Scale LED brightness based on ambient light with a linear curve
    uint8_t led_brightness = MAX_BRIGHTNESS / 2; // Default to mid brightness
    
    if (veml7700_available) {
      float current_lux = veml7700.readLux();
      Serial.print("["); Serial.print(millis()); Serial.print(" ms] Current Lux: ");
      Serial.println(current_lux, 1);
      
      if (current_lux >= MAX_LUX) {
        // Keep at maximum brightness when over the high lux threshold
        led_brightness = MAX_BRIGHTNESS;
      } else if (current_lux <= MIN_LUX) {
        led_brightness = MIN_BRIGHTNESS; // Minimum brightness for very dark environments
      } else {
        // Linear scaling between MIN_BRIGHTNESS and MAX_BRIGHTNESS for lux values between MIN_LUX and MAX_LUX
        led_brightness = MIN_BRIGHTNESS + (current_lux - MIN_LUX) * (MAX_BRIGHTNESS - MIN_BRIGHTNESS) / (MAX_LUX - MIN_LUX);
      }
      
      // Reduce the brightness to 1/2 of the calculated value
      led_brightness = led_brightness / 2;
      
      // Ensure minimum brightness is maintained
      if (led_brightness < MIN_BRIGHTNESS) {
        led_brightness = MIN_BRIGHTNESS;
      }
    }
    
    // Turn on LED for transmission (stays on during entire transmission)
    led_rgb_on(255, 0, 0, led_brightness); // Red LED with brightness based on ambient light
    
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] LED Brightness: ");
    Serial.println(led_brightness);
    
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] Sending packet: ");
    Serial.println(packet);
    
    // Start timing for transmission
    unsigned long startTime = millis();
    
    // Send the packet
    rf95.send((uint8_t *)packet, strlen(packet));
    
    // Wait for transmission to complete with timeout from Gecko_RF95.h
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Waiting for packet to be sent...");
    rf95.waitPacketSent(LORA_TX_TIMEOUT);
    
    // Calculate transmission time
    unsigned long transmitTime = millis() - startTime;
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] Transmission took ");
    Serial.print(transmitTime);
    Serial.println(" ms");
    
    // Turn off LED after transmission is complete
    led_rgb_off();
    
    // Increment packet counter
    packetsSent++;
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] Packet sent successfully! Packets Sent: ");
    Serial.println(packetsSent);
    
    // Update timing
    lastPacketTime = millis();
    
    // Print a separator for readability
    Serial.println("------------------------------");
  }
}

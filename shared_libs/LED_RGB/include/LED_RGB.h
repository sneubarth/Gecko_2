#ifndef LED_RGB_H
#define LED_RGB_H

#include <Arduino.h>
#include <FastLED.h>

// Configuration
#define LED_PIN 1  // GPIO1 for RGB LED data line
#define NUM_LEDS 1 // Single RGB LED
#define LED_TYPE WS2812B // Common RGB LED type (adjust if needed)

// Initialize LED
void led_rgb_init(void);

// Set LED color (red, green, blue), brightness, and duration
void led_rgb_blink(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, uint32_t duration_ms);

// Turn on LED with specified color and brightness (stays on until turned off)
void led_rgb_on(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

// Turn off LED
void led_rgb_off(void);

#endif // LED_RGB_H

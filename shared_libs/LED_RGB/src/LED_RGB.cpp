#include "LED_RGB.h"

// LED array
CRGB leds[NUM_LEDS];

// Initialize LED
void led_rgb_init(void) {
    // Initialize FastLED
    FastLED.addLeds<LED_TYPE, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    
    // Turn off LED initially
    leds[0] = CRGB::Black;
    FastLED.show();
}

// Set LED color (red, green, blue), brightness, and duration
void led_rgb_blink(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, uint32_t duration_ms) {
    // Set color
    leds[0] = CRGB(r, g, b);
    
    // Set brightness
    FastLED.setBrightness(brightness);
    FastLED.show();
    
    // Wait for specified duration if needed
    if (duration_ms > 0) {
        delay(duration_ms);
        
        // Turn off LED after duration
        leds[0] = CRGB::Black;
        FastLED.show();
    }
}

// Turn on LED with specified color and brightness (stays on until turned off)
void led_rgb_on(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    // Set color
    leds[0] = CRGB(r, g, b);
    
    // Set brightness
    FastLED.setBrightness(brightness);
    FastLED.show();
}

// Turn off LED
void led_rgb_off(void) {
    // Turn off LED
    leds[0] = CRGB::Black;
    FastLED.show();
}

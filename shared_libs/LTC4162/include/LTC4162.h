#ifndef LTC4162_H
#define LTC4162_H

#include <Arduino.h>
#include <Wire.h>

enum ltc4162_status_t {
    LTC4162_OK = 0,
    LTC4162_I2C_FAIL = 1,
    LTC4162_TELEMETRY_INVALID = 2
};

class LTC4162 {
public:
    LTC4162();
    bool begin(TwoWire *wire = &Wire);
    ltc4162_status_t readBatteryVoltage(float *vbat_volts);
    bool anyVoltagePresent();
private:
    TwoWire *_wire;
    bool _chargerPresent;
    uint8_t _cellCount;
    bool checkChargerPresence();
    void resetI2CBus();
    uint16_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint16_t value);
};

// Register definitions
#define LTC4162_ADDRESS         0x68
#define REG_VBAT                0x3A
#define REG_SYSTEM_STATUS       0x39
#define REG_CHEM_CELLS          0x43
#define REG_CONFIG_BITS         0x14
#define I2C_RETRY_COUNT         3
#define VBAT_SCALE              192.4e-6  // V/LSB

#endif
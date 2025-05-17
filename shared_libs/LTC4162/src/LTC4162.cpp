#include "LTC4162.h"

LTC4162::LTC4162() {
    _wire = nullptr;
    _chargerPresent = false;
    _cellCount = 4;
}

bool LTC4162::begin(TwoWire *wire) {
    _wire = wire;
    _chargerPresent = false;
    _cellCount = 4; // Default to 4 cells

    // Check if LTC4162 is present
    if (!checkChargerPresence()) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 not detected during init");
        return false;
    }

    // Force telemetry and MPPT on (set bit 2 and 1 in CONFIG_BITS)
    if (!writeRegister(REG_CONFIG_BITS, 0x06)) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] Failed to configure CONFIG_BITS for force_telemetry_on");
        return false;
    }
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] Telemetry forced on, enabling charger control logic");

    // Read CHEM_CELLS
    uint16_t chem_cells = readRegister(REG_CHEM_CELLS);
    if (chem_cells == 0xFFFF || (chem_cells & 0x0F) == 0) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 CHEM_CELLS read failed or invalid, using default Cell Count: 4");
        _cellCount = 4;
    } else {
        _cellCount = chem_cells & 0x0F;
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] LTC4162 CHEM_CELLS Raw: 0x");
        Serial.print(chem_cells, HEX); Serial.print(", Cell Count: "); Serial.println(_cellCount);
        if (_cellCount != 3 && _cellCount != 4) {
            Serial.print("["); Serial.print(millis()); Serial.println(" ms] Warning: Invalid Cell Count, expected 3 or 4, using default 4");
            _cellCount = 4;
        }
    }

    // Read SYSTEM_STATUS for debugging
    uint16_t system_status = readRegister(REG_SYSTEM_STATUS);
    if (system_status == 0xFFFF) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] Failed to read SYSTEM_STATUS");
        return false;
    }
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] SYSTEM_STATUS Raw: 0x");
    Serial.println(system_status, HEX);

    _chargerPresent = true;
    return true;
}

ltc4162_status_t LTC4162::readBatteryVoltage(float *vbat_volts) {
    static bool error_logged = false;
    static float last_voltage = -1.0; // Track last valid voltage
    if (!_chargerPresent) {
        *vbat_volts = 0.0;
        if (!error_logged) {
            Serial.print("["); Serial.print(millis()); Serial.println(" ms] Skipping battery voltage read: LTC4162 not present");
            error_logged = true;
        }
        return LTC4162_I2C_FAIL;
    }

    uint16_t vbat_raw = readRegister(REG_VBAT);
    if (vbat_raw == 0xFFFF) {
        *vbat_volts = 0.0;
        if (!error_logged) {
            Serial.print("["); Serial.print(millis()); Serial.println(" ms] Battery Voltage: 0.00 V (I2C failure)");
            error_logged = true;
        }
        return LTC4162_I2C_FAIL;
    }

    // According to the datasheet, the vbat register has a scaling factor of cell_count × 192.4μV/LSB
    // This means we need to multiply the raw ADC value by both the scale factor and the cell count
    *vbat_volts = vbat_raw * VBAT_SCALE * _cellCount;

    // Log only if voltage changes significantly or on first valid read
    if (abs(*vbat_volts - last_voltage) > 0.1 || last_voltage < 0.0) {
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] VBAT Raw: 0x");
        Serial.print(vbat_raw, HEX); Serial.print(" ("); Serial.print(vbat_raw); Serial.print(")");
        Serial.print(", Cell Count: "); Serial.print(_cellCount);
        Serial.print(", VBAT_SCALE: "); Serial.print(VBAT_SCALE, 8);
        Serial.print(", Calculated Voltage: "); Serial.print(*vbat_volts, 3); 
        Serial.print(", Displayed: "); Serial.print(*vbat_volts, 1); Serial.println(" V");
        last_voltage = *vbat_volts;
    }

    error_logged = false;
    return LTC4162_OK;
}

void LTC4162::resetI2CBus() {
    _wire->end();
    pinMode(SDA, OUTPUT);
    pinMode(SCL, OUTPUT);
    for (int i = 0; i < 10; i++) {
        digitalWrite(SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL, LOW);
        delayMicroseconds(5);
    }
    digitalWrite(SDA, HIGH);
    digitalWrite(SCL, HIGH);
    _wire->begin();
    _wire->setClock(50000);
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] I2C bus reset");
}

uint16_t LTC4162::readRegister(uint8_t reg) {
    if (!_chargerPresent) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] Skipping read: LTC4162 not present");
        return 0xFFFF;
    }
    for (int retry = 0; retry < I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(LTC4162_ADDRESS);
        _wire->write(reg);
        int error = _wire->endTransmission(false);
        if (error == 0) {
            _wire->requestFrom((uint8_t)LTC4162_ADDRESS, (uint8_t)2);
            if (_wire->available() >= 2) {
                uint16_t value = _wire->read() | (_wire->read() << 8);
                return value;
            }
            Serial.print("["); Serial.print(millis()); Serial.print(" ms] I2C read error: Insufficient data, retry ");
            Serial.println(retry + 1);
        } else {
            Serial.print("["); Serial.print(millis()); Serial.print(" ms] I2C write error: Code ");
            Serial.print(error); Serial.print(", retry ");
            Serial.println(retry + 1);
        }
        delay(10);
    }
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] I2C read failed after retries, resetting bus");
    resetI2CBus();
    _chargerPresent = false;
    return 0xFFFF;
}

bool LTC4162::writeRegister(uint8_t reg, uint16_t value) {
    if (!_chargerPresent) {
        Serial.print("["); Serial.print(millis()); Serial.println(" ms] Skipping write: LTC4162 not present");
        return false;
    }
    for (int retry = 0; retry < I2C_RETRY_COUNT; retry++) {
        _wire->beginTransmission(LTC4162_ADDRESS);
        _wire->write(reg);
        _wire->write(value & 0xFF);
        _wire->write(value >> 8);
        int error = _wire->endTransmission();
        if (error == 0) {
            return true;
        }
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] I2C write error: Code ");
        Serial.print(error); Serial.print(", retry ");
        Serial.println(retry + 1);
        delay(10);
    }
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] I2C write failed after retries, resetting bus");
    resetI2CBus();
    _chargerPresent = false;
    return false;
}

bool LTC4162::checkChargerPresence() {
    for (int retry = 0; retry < 5; retry++) {
        _wire->beginTransmission(LTC4162_ADDRESS);
        int error = _wire->endTransmission();
        if (error == 0) {
            _chargerPresent = true;
            Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 detected at 0x68");
            return true;
        }
        Serial.print("["); Serial.print(millis()); Serial.print(" ms] I2C error in checkChargerPresence, error code: ");
        Serial.print(error); Serial.print(", retry ");
        Serial.println(retry + 1);
        delay(50);
    }
    _chargerPresent = false;
    Serial.print("["); Serial.print(millis()); Serial.println(" ms] LTC4162 not detected at 0x68");
    return false;
}

bool LTC4162::anyVoltagePresent() {
    if (!_chargerPresent) return false;
    float vbat_volts;
    ltc4162_status_t vbat_status = readBatteryVoltage(&vbat_volts);
    uint16_t system_status = readRegister(REG_SYSTEM_STATUS);
    bool vin_present = (system_status != 0xFFFF && (system_status & 0x0002));
    return (vbat_status == LTC4162_OK && vbat_volts > 0.0) || vin_present;
}

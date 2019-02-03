#include <VL6180X.h>


// RANGE_SCALER values for 1x, 2x, 3x scaling - see STSW-IMG003 core/src/vl6180x_api.c (ScalerLookUP[])
static uint16_t const ScalerValues[] = {0, 253, 127, 84};


void VL6180X::setAddress(uint8_t new_addr) {
    writeReg(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
    address_ = new_addr;
}

/* Set private registers.
 *
 * See ST application note AN4545, section 9 - "Mandatory : private registers"
 */
void VL6180X::setPrivateRegisters() {
    writeReg(0x207, 0x01);
    writeReg(0x208, 0x01);
    writeReg(0x096, 0x00);
    writeReg(0x097, 0xFD); // RANGE_SCALER = 253
    writeReg(0x0E3, 0x00);
    writeReg(0x0E4, 0x04);
    writeReg(0x0E5, 0x02);
    writeReg(0x0E6, 0x01);
    writeReg(0x0E7, 0x03);
    writeReg(0x0F5, 0x02);
    writeReg(0x0D9, 0x05);
    writeReg(0x0DB, 0xCE);
    writeReg(0x0DC, 0x03);
    writeReg(0x0DD, 0xF8);
    writeReg(0x09F, 0x00);
    writeReg(0x0A3, 0x3C);
    writeReg(0x0B7, 0x00);
    writeReg(0x0BB, 0x3C);
    writeReg(0x0B2, 0x09);
    writeReg(0x0CA, 0x09);
    writeReg(0x198, 0x01);
    writeReg(0x1B0, 0x17);
    writeReg(0x1AD, 0x00);
    writeReg(0x0FF, 0x05);
    writeReg(0x100, 0x05);
    writeReg(0x199, 0x05);
    writeReg(0x1A6, 0x1B);
    writeReg(0x1AC, 0x3E);
    writeReg(0x1A7, 0x1F);
    writeReg(0x030, 0x00);
}

/*
 * Configure some settings for the sensor's default behavior from AN4545 -
 * "Recommended : Public registers" and "Optional: Public registers"
 *
 * Note that this function does not set up GPIO1 as an interrupt output as
 * suggested, though you can do so by calling:
 * writeReg(SYSTEM__MODE_GPIO1, 0x10);
 */
void VL6180X::setDefaults() {
    // "Recommended : Public registers"

    // readout__averaging_sample_period = 48
    writeReg(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);

    // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to Table 14 in datasheet)
    writeReg(SYSALS__ANALOGUE_GAIN, 0x46);

    // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
    writeReg(SYSRANGE__VHV_REPEAT_RATE, 0xFF);

    // sysals__integration_period = 99 (100 ms)
    // AN4545 incorrectly recommends writing to register 0x040; 0x63 should go in the lower byte, which is register 0x041.
    writeReg16Bit(SYSALS__INTEGRATION_PERIOD, 0x0063);

    // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
    writeReg(SYSRANGE__VHV_RECALIBRATE, 0x01);


    // "Optional: Public registers"

    // sysrange__intermeasurement_period = 9 (100 ms)
    writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);

    // sysals__intermeasurement_period = 49 (500 ms)
    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);

    // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
    writeReg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);


    // Reset other settings to power-on defaults

    // sysrange__max_convergence_time = 49 (49 ms)
    writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);

    // disable interleaved mode
    writeReg(INTERLEAVED_MODE__ENABLE, 0);

    // reset range scaling factor to 1x
    setScaling(1);
}

void VL6180X::begin() {
    // Store part-to-part range offset so it can be adjusted if scaling is changed
    ptp_offset_ = readReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET);

    if (readReg(SYSTEM__FRESH_OUT_OF_RESET) == 1) {
        scaling_ = 1;
        setPrivateRegisters();
        writeReg(SYSTEM__FRESH_OUT_OF_RESET, 0);
    } else {
        // Sensor has already been initialized, so try to get scaling settings by
        // reading registers.

        uint16_t s = readReg16Bit(RANGE_SCALER);

        if (s == ScalerValues[3]) { scaling_ = 3; }
        else if (s == ScalerValues[2]) { scaling_ = 2; }
        else { scaling_ = 1; }

        // Adjust the part-to-part range offset value read earlier to account for
        // existing scaling. If the sensor was already in 2x or 3x scaling mode,
        // precision will be lost calculating the original (1x) offset, but this can
        // be resolved by resetting the sensor and Arduino again.
        ptp_offset_ *= scaling_;
    }

    setDefaults();
}

void VL6180X::setScaling(uint8_t new_scaling) {
    uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT

    // do nothing if scaling value is invalid
    if (new_scaling < 1 || new_scaling > 3) { return; }

    scaling_ = new_scaling;
    writeReg16Bit(RANGE_SCALER, ScalerValues[scaling_]);

    // apply scaling on part-to-part offset
    writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, ptp_offset_ / scaling_);

    // apply scaling on CrossTalkValidHeight
    writeReg(VL6180X::SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling_);

    // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

    // enable early convergence estimate only at 1x scaling
    uint8_t rce = readReg(VL6180X::SYSRANGE__RANGE_CHECK_ENABLES);
    writeReg(VL6180X::SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling_ == 1));
}

uint8_t VL6180X::readRangeSingle() {
    writeReg(SYSRANGE__START, 0x01);
    return readRangeContinuous();
}

uint16_t VL6180X::readAmbientSingle() {
    writeReg(SYSALS__START, 0x01);
    return readAmbientContinuous();
}

void VL6180X::startRangeContinuous(uint16_t period) {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSRANGE__START, 0x03);
}

void VL6180X::startAmbientContinuous(uint16_t period) {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSALS__START, 0x03);
}

void VL6180X::startInterleavedContinuous(uint16_t period) {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    writeReg(INTERLEAVED_MODE__ENABLE, 1);
    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSALS__START, 0x03);
}


void VL6180X::stopContinuous() {
    writeReg(SYSRANGE__START, 0x01);
    writeReg(SYSALS__START, 0x01);

    writeReg(INTERLEAVED_MODE__ENABLE, 0);
}

uint8_t VL6180X::readRangeContinuous() {
    uint16_t millis_start = millis();
    while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x04) == 0) {
        if (io_timeout_ > 0 && ((uint16_t) millis() - millis_start) > io_timeout_) {
            did_timeout_ = true;
            return 255;
        }
    }

    uint8_t range = readReg(RESULT__RANGE_VAL);
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t VL6180X::readAmbientContinuous() {
    uint16_t millis_start = millis();
    while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x20) == 0) {
        if (io_timeout_ > 0 && ((uint16_t) millis() - millis_start) > io_timeout_) {
            did_timeout_ = true;
            return 0;
        }
    }

    uint16_t ambient = readReg16Bit(RESULT__ALS_VAL);
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x02);

    return ambient;
}

bool VL6180X::timeoutOccurred() {
    bool tmp = did_timeout_;
    did_timeout_ = false;
    return tmp;
}


void VL6180X::writeReg(uint16_t reg, uint8_t value) {
    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    wire_.write(value);
    last_status = wire_.endTransmission();
}

void VL6180X::writeReg16Bit(uint16_t reg, uint16_t value) {
    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    wire_.write((value >> 8) & 0xff);  // value high byte
    wire_.write(value & 0xff);         // value low byte
    last_status = wire_.endTransmission();
}

void VL6180X::writeReg32Bit(uint16_t reg, uint32_t value) {
    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    wire_.write((value >> 24) & 0xff); // value highest byte
    wire_.write((value >> 16) & 0xff);
    wire_.write((value >> 8) & 0xff);
    wire_.write(value & 0xff);         // value lowest byte
    last_status = wire_.endTransmission();
}

uint8_t VL6180X::readReg(uint16_t reg) {
    uint8_t value;

    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    last_status = wire_.endTransmission();

    wire_.requestFrom(address_, (uint8_t) 1);
    value = wire_.read();
    wire_.endTransmission();

    return value;
}

uint16_t VL6180X::readReg16Bit(uint16_t reg) {
    uint16_t value;

    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    last_status = wire_.endTransmission();

    wire_.requestFrom(address_, (uint8_t) 2);
    value = (uint16_t) wire_.read() << 8; // value high byte
    value |= wire_.read();               // value low byte
    wire_.endTransmission();

    return value;
}

uint32_t VL6180X::readReg32Bit(uint16_t reg) {
    uint32_t value;

    wire_.beginTransmission(address_);
    wire_.write((reg >> 8) & 0xff);  // reg high byte
    wire_.write(reg & 0xff);         // reg low byte
    last_status = wire_.endTransmission();

    wire_.requestFrom(address_, (uint8_t) 4);
    value = (uint32_t) wire_.read() << 24;  // value highest byte
    value |= (uint32_t) wire_.read() << 16;
    value |= (uint16_t) wire_.read() << 8;
    value |= wire_.read();                 // value lowest byte
    wire_.endTransmission();

    return value;
}

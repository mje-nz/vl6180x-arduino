#ifndef VL6180X_h
#define VL6180X_h

#include <Arduino.h>


// To use the i2c_t3 library instead of Wire, uncomment this or add -DVL6180X_USE_TEENSY to your build
//#define VL6180X_USE_I2C_T3
#ifdef VL6180X_USE_I2C_T3
#include <i2c_t3.h>
typedef i2c_t3 TwoWire;
#else
#include <Wire.h>
#endif


class VL6180X {
public:
    // register addresses
    enum regAddr {
        IDENTIFICATION__MODEL_ID = 0x000,
        IDENTIFICATION__MODEL_REV_MAJOR = 0x001,
        IDENTIFICATION__MODEL_REV_MINOR = 0x002,
        IDENTIFICATION__MODULE_REV_MAJOR = 0x003,
        IDENTIFICATION__MODULE_REV_MINOR = 0x004,
        IDENTIFICATION__DATE_HI = 0x006,
        IDENTIFICATION__DATE_LO = 0x007,
        IDENTIFICATION__TIME = 0x008, // 16-bit

        SYSTEM__MODE_GPIO0 = 0x010,
        SYSTEM__MODE_GPIO1 = 0x011,
        SYSTEM__HISTORY_CTRL = 0x012,
        SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014,
        SYSTEM__INTERRUPT_CLEAR = 0x015,
        SYSTEM__FRESH_OUT_OF_RESET = 0x016,
        SYSTEM__GROUPED_PARAMETER_HOLD = 0x017,

        SYSRANGE__START = 0x018,
        SYSRANGE__THRESH_HIGH = 0x019,
        SYSRANGE__THRESH_LOW = 0x01A,
        SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B,
        SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C,
        SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
        SYSRANGE__CROSSTALK_VALID_HEIGHT = 0x021,
        SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022, // 16-bit
        SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024,
        SYSRANGE__RANGE_IGNORE_VALID_HEIGHT = 0x025,
        SYSRANGE__RANGE_IGNORE_THRESHOLD = 0x026, // 16-bit
        SYSRANGE__MAX_AMBIENT_LEVEL_MULT = 0x02C,
        SYSRANGE__RANGE_CHECK_ENABLES = 0x02D,
        SYSRANGE__VHV_RECALIBRATE = 0x02E,
        SYSRANGE__VHV_REPEAT_RATE = 0x031,

        SYSALS__START = 0x038,
        SYSALS__THRESH_HIGH = 0x03A,
        SYSALS__THRESH_LOW = 0x03C,
        SYSALS__INTERMEASUREMENT_PERIOD = 0x03E,
        SYSALS__ANALOGUE_GAIN = 0x03F,
        SYSALS__INTEGRATION_PERIOD = 0x040,

        RESULT__RANGE_STATUS = 0x04D,
        RESULT__ALS_STATUS = 0x04E,
        RESULT__INTERRUPT_STATUS_GPIO = 0x04F,
        RESULT__ALS_VAL = 0x050, // 16-bit
        RESULT__HISTORY_BUFFER_0 = 0x052, // 16-bit
        RESULT__HISTORY_BUFFER_1 = 0x054, // 16-bit
        RESULT__HISTORY_BUFFER_2 = 0x056, // 16-bit
        RESULT__HISTORY_BUFFER_3 = 0x058, // 16-bit
        RESULT__HISTORY_BUFFER_4 = 0x05A, // 16-bit
        RESULT__HISTORY_BUFFER_5 = 0x05C, // 16-bit
        RESULT__HISTORY_BUFFER_6 = 0x05E, // 16-bit
        RESULT__HISTORY_BUFFER_7 = 0x060, // 16-bit
        RESULT__RANGE_VAL = 0x062,
        RESULT__RANGE_RAW = 0x064,
        RESULT__RANGE_RETURN_RATE = 0x066, // 16-bit
        RESULT__RANGE_REFERENCE_RATE = 0x068, // 16-bit
        RESULT__RANGE_RETURN_SIGNAL_COUNT = 0x06C, // 32-bit
        RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070, // 32-bit
        RESULT__RANGE_RETURN_AMB_COUNT = 0x074, // 32-bit
        RESULT__RANGE_REFERENCE_AMB_COUNT = 0x078, // 32-bit
        RESULT__RANGE_RETURN_CONV_TIME = 0x07C, // 32-bit
        RESULT__RANGE_REFERENCE_CONV_TIME = 0x080, // 32-bit

        RANGE_SCALER = 0x096, // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

        READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A,
        FIRMWARE__BOOTUP = 0x119,
        FIRMWARE__RESULT_SCALER = 0x120,
        I2C_SLAVE__DEVICE_ADDRESS = 0x212,
        INTERLEAVED_MODE__ENABLE = 0x2A3,
    };

    /* The status of the last I2C write transmission.
     * 
     * See the [Wire.endTransmission() documentation](http://arduino.cc/en/Reference/WireEndTransmission) for possible values.
     */
    uint8_t last_status = 0; // status of last I2C transmission

    VL6180X(TwoWire& theWire=Wire)
      : wire_(theWire) {
    }

    /* Initialize sensor.
     *
     * Call Wire.begin() first.
     */
    void begin();

    /* Change the I&sup2;C slave device address of the VL6180X to the given value (7-bit). */
    void setAddress(uint8_t new_addr);

    /* Set range scaling factor.
     *
     * The sensor uses 1x scaling by default, giving range measurements in units of mm.  Increasing the scaling to 2x or
     * 3x makes it give raw values in units of 2 mm or 3 mm instead.  In other words, a bigger scaling factor increases
     * the sensor's potential maximum range but reduces its resolution.
     *
     * Implemented using ST's VL6180X API as a reference (STSW-IMG003); see VL6180x_UpscaleSetScaling() in
     * vl6180x_api.c.
     */
    void setScaling(uint8_t new_scaling);

    /* Return the current range scaling factor. */
    inline uint8_t getScaling() { return scaling_; }

    /* Perform a single-shot ranging measurement and return the raw reading. */
    uint8_t readRangeSingle();

    /* Perform a single-shot ranging measurement and returns the reading in millimeters, taking the range scaling setting into account. */
    inline uint16_t readRangeSingleMillimeters() { return (uint16_t) scaling_ * readRangeSingle(); }

    /* Perform a single-shot ambient light measurement. */
    uint16_t readAmbientSingle();

    /* Start continuous ranging measurements with the given period in ms (10 ms resolution).
     *
     * The period must be greater than the time it takes to perform a measurement.  See section 2.4.4 ("Continuous mode
     * limits") in the datasheet for details.
     */
    void startRangeContinuous(uint16_t period=100);

    /* Start continuous ambient light measurements with the given period in ms (10 ms resolution).
     *
     * The period must be greater than the time it takes to perform a measurement.  See section 2.4.4 ("Continuous mode
     * limits") in the datasheet for details.
     */
    void startAmbientContinuous(uint16_t period=500);

    /* Start continuous interleaved measurements with the given period in ms (10 ms resolution).
     *
     * In this mode, each ambient light measurement is immediately followed by a range measurement.  The datasheet
     * recommends using this mode instead of running "range and ALS continuous modes simultaneously (i.e.
     * asynchronously)".
     *
     * The period must be greater than the time it takes to perform both measurements.  See section 2.4.4 ("Continuous
     * mode limits") in the datasheet for details.
     */
    void startInterleavedContinuous(uint16_t period=500);

    /* Stop continuous mode.
     *
     * This will actually start a single measurement of range and/or ambient light if continuous mode is not active, so
     * it's a good idea to wait a few hundred ms after calling this function to let that complete before starting
     * continuous mode again or taking a reading.
     */
    void stopContinuous();

    /* Return a raw range reading when continuous mode is active.
     *
     * readRangeSingle() also calls this function after starting a single-shot range measurement.
     */
    uint8_t readRangeContinuous();

    /* Return a range reading in millimeters, taking the range scaling setting into account, when continuous mode is active. */
    inline uint16_t readRangeContinuousMillimeters() { return (uint16_t) scaling_ * readRangeContinuous(); }
    
    /* Return an ambient light reading when continuous mode is activated.
     *
     * readAmbientSingle() also calls this function after starting a single-shot ambient light measurement.
     */
    uint16_t readAmbientContinuous();

    /* Set timeout period in milliseconds after which the read functions will abort if the sensor is not ready.
     *
     * A value of 0 disables the timeout.
     */
    inline void setTimeout(uint16_t timeout) { io_timeout_ = timeout; }

    /* Return the current timeout period setting. */
    inline uint16_t getTimeout() { return io_timeout_; }

    /* Return whether a read timeout has occurred since the last call to timeoutOccurred(). */
    bool timeoutOccurred();

    /*
     * Write the given value to an 8-bit sensor register.
     * 
     * Register address constants are defined by the regAddr enumeration type in VL6180X.h.
     * Example use: `sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);`
     */
    void writeReg(uint16_t reg, uint8_t value);
    /* Write the given value to a 16-bit sensor register */
    void writeReg16Bit(uint16_t reg, uint16_t value);
    /* Write the given value to a 32-bit sensor register */
    void writeReg32Bit(uint16_t reg, uint32_t value);
    /* Read an 8-bit sensor register. */
    uint8_t readReg(uint16_t reg);
    /* Read a 16-bit sensor register. */
    uint16_t readReg16Bit(uint16_t reg);
    /* Read a 32-bit sensor register. */
    uint32_t readReg32Bit(uint16_t reg);

private:
    void setPrivateRegisters();
    void setDefaults();
    TwoWire &wire_;
    uint8_t address_ = 0b0101001;
    uint8_t scaling_ = 0;
    uint8_t ptp_offset_ = 0;
    uint16_t io_timeout_ = 0;
    bool did_timeout_ = false;
};

#endif




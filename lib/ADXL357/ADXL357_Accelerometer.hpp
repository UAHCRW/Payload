#ifndef ADXL356_ACCELEROMETER_HPP
#define ADXL356_ACCELEROMETER_HPP

#include "Arduino.h"
#include "SPI.h"
#include "logger.hpp"
#include <logger.hpp>
#include <stdint.h>

#define ANALOG_DEV_ID         0xAD
#define DEV_MEMS_ID           0x1D
#define DEVICE_ID             0xED
#define RANGE_SCALE_FACTOR_40 78.0  // micro g / LSB
#define RANGE_SCALE_FACTOR_20 39    // micro g / LSB
#define RANGE_SCALE_FACTOR_10 19.5  // micro g / LSB
#define TEMP_SCALE_FACTOR     -9.05 // LSB / deg C
#define MAX_FIFO_SAMPLES      96
namespace ADXL357
{
    // General Register Values
    enum REGISTER : uint16_t
    {
        DEV_ID_AD    = 0x00,
        DEV_ID_MEM   = 0X01,
        DEV_ID       = 0X02,
        REV_ID       = 0X03,
        STATUS       = 0X04,
        FIFO_ENTRIES = 0X05,
        TEMP2        = 0X06,
        TEMP1        = 0X07,
        XDATA3       = 0X08,
        XDATA2       = 0X09,
        XDATA1       = 0X0A,
        YDATA3       = 0X0B,
        YDATA2       = 0X0C,
        YDATA1       = 0X0D,
        ZDATA3       = 0X0E,
        ZDATA2       = 0X0F,
        ZDATA1       = 0X10,
        FIFO_DATA    = 0X11,
        OFFSET_X_H   = 0X1E,
        OFFSET_X_L   = 0X1F,
        OFFSET_Y_H   = 0X20,
        OFFSET_Y_L   = 0X21,
        OFFSET_Z_H   = 0X22,
        OFFSET_Z_L   = 0X23,
        ACT_EN       = 0X24,
        ACT_THRESH_H = 0X25,
        ACT_THRESH_L = 0X26,
        ACT_COUNT    = 0X27,
        FILTER       = 0X28,
        FIFO_SAMPLES = 0X29,
        INT_MAP      = 0X2A,
        SYNC         = 0X2B,
        ACCEL_RANGE  = 0X2C,
        POWER_CTL    = 0X2D,
        SELF_TEST    = 0X2E,
        RESET        = 0X2F,
    };

    // Register Values for High pass filter
    // These values are multiplied by the output data rate (ODR)
    // Format for 1.5545x10^-4 X ODR -> HP_1_5545 where _ represents a decimal
    enum HIGH_PASS_FILTER : uint16_t
    {
        NO_FILTER = 0x00,
        HP_24_7   = 0x01,
        HP_6_2084 = 0x02,
        HP_1_5545 = 0x03,
        HP_3862   = 0x04,
        HP_0954   = 0x05,
        HP_0238   = 0x6,
    };

    // Output Data Rate (ODR) -> Low Pass Filter (LP) "_" in a number represents a decimal
    // All values in Hz
    enum ODR_LPF : uint16_t
    {
        ODR_4000_LP_1000    = 0x00,
        ODR_2000_LP_500     = 0x01,
        ODR_1000_LP_250     = 0x02,
        ODR_500_LP_125      = 0x03,
        ODR_250_LP_62_5     = 0x04,
        ODR_125_LP_31_25    = 0x05,
        ODR_62_5_LP_15_625  = 0x06,
        ODR_31_25_LP_7_813  = 0x07,
        ODR_15_625_LP_3_906 = 0x08,
        ODR_7_813_LP_1_953  = 0x09,
        ODR_3_906_LP_0_977  = 0x0A,
    };

    // MISC
    enum MISC : uint8_t
    {
        RESET_CODE = 0X52,
    };

    enum I2CSpeed : uint8_t
    {
        LOW_SPEED  = 0,
        HIGH_SPEED = 1,
    };

    enum AccelRange : uint8_t
    {
        TEN    = 0x1,
        TWENTY = 0x2,
        FORTY  = 0x3,
    };

    enum InterruptPolarity : uint8_t
    {
        ACTIVE_LOW  = 0,
        ACTIVE_HIGH = 1,
    };

    struct AccelerometerData
    {
        double x;
        double y;
        double z;
        double temp;

        AccelerometerData() : x(0.0), y(0.0), z(0.0), temp(0.0) {}
    };

    struct AccelerometerRawData
    {
        uint32_t x;
        uint32_t y;
        uint32_t z;
        uint32_t temp;

        AccelerometerRawData() : x(0), y(0), z(0), temp(0) {}
    };

    struct AccelerometerConfig
    {
        AccelerometerData sensorBias;
        ODR_LPF odrLpfSetting;
        HIGH_PASS_FILTER hpfSetting;
        AccelRange sensorRange;
        I2CSpeed i2cSpeed;
        InterruptPolarity interruptPolarity;

        AccelerometerConfig()
            : odrLpfSetting(ODR_LPF::ODR_250_LP_62_5), hpfSetting(HIGH_PASS_FILTER::NO_FILTER),
              sensorRange(AccelRange::FORTY), i2cSpeed(I2CSpeed::HIGH_SPEED),
              interruptPolarity(InterruptPolarity::ACTIVE_HIGH)
        {
        }
    };

    class Accelerometer final
    {
        public:
        Accelerometer(uint8_t chipSelectPin, uint8_t clock, AccelerometerConfig& config);
        ~Accelerometer(){};

        // API to get measurements from sensor
        ////////////////////////////////////////////////////////////
        /// \brief Starts the sensor and ensures secure connection
        /// \returns True if sensor is found, false if it is not
        bool begin();

        /// \brief Gets the Accelerometer Measurement along the X-Axis
        double getAccelX() { return readAccelerationReg(REGISTER::XDATA3); }

        /// \brief Gets the Accelerometer Measurement along the Y-Axis
        double getAccelY() { return readAccelerationReg(REGISTER::YDATA3); }

        /// \brief Gets the Accelerometer Measurement along the Z-Axis
        double getAccelZ() { return readAccelerationReg(REGISTER::ZDATA3); }

        /// \brief Gets the raw value for the acceleration register
        uint32_t getRawAccelX() { return readRawAccelerationReg(REGISTER::XDATA3); }

        /// \brief Gets the raw value for the acceleration register
        uint32_t getRawAccelY() { return readRawAccelerationReg(REGISTER::YDATA3); }

        /// \brief Gets the raw value for the acceleration register
        uint32_t getRawAccelZ() { return readRawAccelerationReg(REGISTER::ZDATA3); }

        /// \brief Gets the temperature measurement
        double getTemperature() { return (double)((int16_t)(getRawTemperature()) * TEMP_SCALE_FACTOR + 25.0); }

        /// \brief Gets the raw value for the temperature register
        uint16_t getRawTemperature();

        /// \brief Gets the sensor state, Accelerometer measurements in X, Y, Z, and temperatures
        AccelerometerData getState();
        AccelerometerData getState(AccelerometerRawData& rawData);

        /// \brief Gets the sensor state but leaves them in the form just as they was received from the register
        AccelerometerRawData getRawState();

        /// \brief Sets the bias along each sensor axis, temperature is ignored
        void setAccelerometerBias(AccelerometerData& data);
        AccelerometerData getAccelerometerBias();

        /// \brief Resets the accelerometer
        void reset() { writeSingleRegister(REGISTER::RESET, MISC::RESET_CODE); }

        /// \brief Gets the Analog devices ID
        uint8_t getAnalogDevicesID();

        /// \brief Gets the analog devices MEMS ID
        uint8_t getAnalogDevicesMemsID();

        /// \brief Gets the Device ID
        uint8_t getDeviceId();

        /// \brief Gets the product revision ID
        uint8_t getProductRevisionID();

        /// \brief Prints the sensor configuration similiar to the format found in settings.cpp
        void printSensorConfiguration();

        /// \brief Get the configuration of the accelerometer by reading registers
        AccelerometerConfig getAccelerometerConfig();

        /// \brief Reads status register to determine if there are valid FIFO samples in the FIFO buffer
        bool isDataReady();

        /// \brief Reads staus register to determine if the fifo is full
        bool isFifoFull();

        /// \brief Reads status register to determine if the fifo has overflowed
        bool hasFifoOverrun();

        /// \brief Gets the number of samples contained in the FIFO buffer
        uint8_t numValidFifoSamples();

        private:
        void select();
        void deselect();
        uint32_t readRawAccelerationReg(uint8_t reg);
        double readAccelerationReg(uint8_t reg);

        // Read Registers
        /////////////////////////////////////////////
        void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes);
        void readSingleRegister(uint8_t reg, uint8_t& data);

        // Write Registers
        /////////////////////////////////////////////
        void writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes);
        void writeSingleRegister(uint8_t reg, uint8_t value);

        // Configure Accelerometer
        /////////////////////////////////////////////
        // Power Control Register
        void setPwrControlReg(bool drdyOn, bool tempOn, bool standby);
        void getPwrControlReg(bool& drdyOn, bool& tempOn, bool& standby);

        // I2C config, AccelRange, Interrupt Polarity regster
        /////////////////////////////////////////////
        void setI2CInterruptRangeReg(I2CSpeed i2cSpeed, InterruptPolarity polarity, AccelRange range);
        void getI2CInterruptRangeReg(I2CSpeed& i2cSpeed, InterruptPolarity& polarity, AccelRange& range);

        // Filter settings and output data rate register
        /////////////////////////////////////////////
        void setFilterSettings(ODR_LPF odrLpf, HIGH_PASS_FILTER hpf);
        void getFilterSettings(ODR_LPF& odrLpf, HIGH_PASS_FILTER& hpf);

        // FIFO Registers
        /////////////////////////////////////////////

        /// \brief Gets the max number of samples stored in the FIFO
        uint8_t getFifoSamples();

        /// \brief Sets the max number of samples stored in the FIFO
        void setFifoSamples(uint8_t numSamples) { writeSingleRegister(REGISTER::FIFO_SAMPLES, numSamples); }

        /// \brief The number of valid entries in the FIFO
        uint8_t getFifoValidSamples();

        /// \brief Gets a complete accelerometer measurement from the FIFO
        void getFifoData(double& x, double& y, double& z);

        // Status Register
        /////////////////////////////////////////////
        void getStatus(bool& nvmBusy, bool& activity, bool& fifoOverrun, bool& fifoFull, bool& dataRdy);

        // Setup
        /////////////////////////////////////////////
        void setRangeScaleFactor();

        // Misc
        /////////////////////////////////////////////
        /// \brief Converts a value from the register in units LSB to a useable reading in G
        double convertReadingScaledToUnscaled(uint32_t reading);

        /// \brief Converts a value in G to a value for the register in units LSB
        uint32_t convertReadingUnscaledToScaled(double reading);

        // Members
        /////////////////////////////////////////////
        uint8_t chipSelect_;
        uint32_t spiClockSpeed_;
        AccelerometerConfig config_;
        float rangeScaleFactor_;
    };

};
#endif

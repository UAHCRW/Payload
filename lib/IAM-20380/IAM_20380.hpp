#ifndef IAM_20380_HEADER
#define IAM_20380_HEADER

#include "SPI.h"
#include "logger.hpp"
#include <logger.hpp>
#include <stdint.h>

namespace IAM20380
{
#define IAM_20380_WHO_AM_I                    0xB5
#define IAM_20380_FIFO_EMPTY_FLAG             0xFF
#define IAM_20380_RANGE_SCALE_FACTOR_250_DPS  131
#define IAM_20380_RANGE_SCALE_FACTOR_500_DPS  65.5
#define IAM_20380_RANGE_SCALE_FACTOR_1000_DPS 32.8
#define IAM_20380_RANGE_SCALE_FACTOR_2000_DPS 16.4
#define IAM_20380_TEMP_SENSITIVITY_FACTOR     326.8

    enum class Register : uint16_t
    {
        SELF_TEST_X_GYRO  = 0x00,
        SELF_TEST_Y_GYRO  = 0x01,
        SELF_TEST_Z_GYRO  = 0x02,
        XG_OFFS_USRH      = 0x13,
        XG_OFFS_USRL      = 0x14,
        YG_OFFS_USRH      = 0x15,
        YG_OFFS_USRL      = 0x16,
        ZG_OFFS_USRH      = 0x17,
        ZG_OFFS_USRL      = 0x18,
        SMPLRT_DIV        = 0x19,
        CONFIG            = 0x1A,
        GYRO_CONFIG       = 0x1B,
        LP_MODE_CFG       = 0x1E,
        FIFO_EN           = 0x23,
        FSYNC_INT         = 0x36,
        INT_PIN_CFG       = 0x37,
        INT_ENABLE        = 0x38,
        INT_STATUS        = 0x3A,
        TEMP_OUT_H        = 0x41,
        TEMP_OUT_L        = 0x42,
        GYRO_XOUT_H       = 0x43,
        GYRO_XOUT_L       = 0x44,
        GYRO_YOUT_H       = 0x45,
        GYRO_YOUT_L       = 0x46,
        GYRO_ZOUT_H       = 0x47,
        GYRO_ZOUT_L       = 0x48,
        SIGNAL_PATH_RESET = 0x68,
        USER_CTRL         = 0x6A,
        PWR_MGMT_1        = 0x6B,
        PWR_MGMT_2        = 0x6C,
        FIFO_COUNTH       = 0x72,
        FIFO_COUNTL       = 0x73,
        FIFO_R_W          = 0x74,
        WHO_AM_I          = 0x75,
    };

    enum class GyroRange : uint8_t
    {
        DPS_250  = 0x00,
        DPS_500  = 0x01,
        DPS_1000 = 0x02,
        DPS_2000 = 0x03,
    };

    enum class PowerMode : uint8_t
    {
        HIGH_POWER = 0,
        LOW_POWER  = 1,
    };

    enum class CommunicationProtocol : uint8_t
    {
        BOTH_I2C_SPI = 0,
        SPI_ONLY     = 1,
    };

    enum class AxisState : uint8_t
    {
        ENABLED  = 0,
        DISABLED = 1,
    };

    struct GyroData
    {
        double x;
        double y;
        double z;
        double temp;
        uint16_t xRaw;
        uint16_t yRaw;
        uint16_t zRaw;
        uint16_t tempRaw;

        GyroData() : x(0.0), y(0.0), z(0.0), temp(0.0), xRaw(0), yRaw(0), zRaw(0), tempRaw(0){};
    };

    struct GyroscopeConfig
    {
        GyroData bias;
        AxisState xAxisState;
        AxisState yAxisState;
        AxisState zAxisState;
        bool fifoEnabled;
        GyroRange range;
        bool i2cDisabled;
        bool sleepEnabled;
        bool inStandby;
        bool tempDisabled;
        uint8_t sampleRateDivider;

        GyroscopeConfig()
            : xAxisState(AxisState::ENABLED), yAxisState(AxisState::ENABLED), zAxisState(AxisState::ENABLED),
              fifoEnabled(false), range(GyroRange::DPS_2000), i2cDisabled(false), sleepEnabled(false), inStandby(false),
              sampleRateDivider(0){};
    };

    class Gyroscope final
    {
        public:
        Gyroscope(){};
        Gyroscope(uint8_t chipSelectPin, uint32_t clock, GyroscopeConfig& config);
        ~Gyroscope(){};

        // API to get measurements from sensor
        ////////////////////////////////////////////////////////////
        /// \brief Starts the sensor and ensures secure connection
        /// \returns True if sensor is found, false if it is not
        bool begin();

        /// \brief Gets the gyrometer measurement along the x - axis
        double getGyroX() { return readGyrometerReg(Register::GYRO_XOUT_H); }

        /// \brief Gets the gyrometer measurement along the y - axis
        double getGyroY() { return readGyrometerReg(Register::GYRO_YOUT_H); }

        /// \brief Gets the gyrometer measurement along the z - axis
        double getGyroZ() { return readGyrometerReg(Register::GYRO_ZOUT_H); }

        /// \brief Gets the raw gyrometer measurement along the x - axis as is read from the register
        uint16_t getRawGyroX() { return readRawGyrometerReg(Register::GYRO_XOUT_H); }

        /// \brief Gets the raw gyrometer measurement along the y - axis as is read from the register
        uint16_t getRawGyroY() { return readRawGyrometerReg(Register::GYRO_YOUT_H); }

        /// \brief Gets the raw gyrometer measurement along the z - axis as is read from the register
        uint16_t getRawGyroZ() { return readRawGyrometerReg(Register::GYRO_ZOUT_H); }

        /// \brief Gets the temperature measurement
        double getTemperature() { return (double)(getRawTemperature() / IAM_20380_TEMP_SENSITIVITY_FACTOR + 25.0); }

        /// \brief Gets the raw temperature value as is read from the register
        uint16_t getRawTemperature();

        /// \brief Gets the state of the gyrometer
        void getState(GyroData& data);

        /// \brief Gets the state of the gyrometer from the first FIFO sample
        void getStateFromFifo(GyroData& data);

        /// \brief Gets the state from the last sample in the FIFO buffer
        void getStateFromLastFifoSample(GyroData& data);

        /// \brief Set the gyro biases
        void setGyroBias(GyroData& bias);

        /// \brief Gets the gyro biases being used by the sensor
        GyroData getGyroBias();

        /// \brief Gets the current configuration of the gyrometer
        GyroscopeConfig getGyroConfig();

        /// \brief Resets the gyrometer
        void reset() { setPowerManagement1Reg(true, false, false, false); }

        /// \brief Resets the FIFO buffer
        void resetFifo() { setUserControlRegister(true, false, true, false); }

        /// \brief Gets the value of the Who Am I register to ensure communication is valid
        uint16_t getWhoAmIReg() { return readSingleRegister(Register::WHO_AM_I); }

        bool validateWhoAmIReg() { return (getWhoAmIReg() == IAM_20380_WHO_AM_I); }

        void printGryoConfiguration();

        uint16_t getNumBytesInFifo();

        private:
        void select();
        void deselect();
        uint32_t readRawGyrometerReg(Register reg);
        double readGyrometerReg(Register reg) { return convertReadingRawToUseable(readRawGyrometerReg(reg)); }

        // Read Registers
        /////////////////////////////////////////////
        void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes);
        void readRegisters(Register reg, uint8_t* buff, uint8_t bytes) { readRegisters((uint8_t)(reg), buff, bytes); }
        uint8_t readSingleRegister(uint8_t reg);
        uint8_t readSingleRegister(Register reg) { return readSingleRegister((uint8_t)(reg)); }

        // Write Registers
        /////////////////////////////////////////////
        void writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes);
        void writeRegisters(Register reg, uint8_t* buff, uint8_t bytes) { writeRegisters((uint8_t)(reg), buff, bytes); }
        void writeSingleRegister(uint8_t reg, uint8_t value);
        void writeSingleRegister(Register reg, uint8_t data) { writeSingleRegister((uint8_t)(reg), data); }

        /// Sample Rate Divider
        /////////////////////////////////////////////
        // Sample rate divide equation 1000 Hz / ( 1 + SMPLRT_DIV)
        void setSampleRateDividerReg(uint8_t val) { writeSingleRegister(Register::SMPLRT_DIV, val); }
        uint8_t getSampleRateDividerReg() { return readSingleRegister(Register::SMPLRT_DIV); }

        /// Gyroscop Configuration Register
        /////////////////////////////////////////////
        void setGyroConfigReg(GyroRange range, bool dlpfBypass);
        void getGyroConfigReg(GyroRange& range, bool& dlpfBypass);

        /// User Control Register
        /////////////////////////////////////////////
        void setUserControlRegister(bool fifoEnable, bool i2cDisable, bool fifoReset, bool sigCondReset);
        void getUserControlRegister(bool& fifoEnable, bool& i2cDisable, bool& fifoReset, bool& sigCondReset);

        // Power Management 1 Register
        /////////////////////////////////////////////
        void setPowerManagement1Reg(bool reset, bool sleep, bool standby, bool tempDisable);
        void getPowerManagement1Reg(bool& reset, bool& sleep, bool& standby, bool& tempDisable);

        // Power Management 2 Register
        void setPowerManagement2Reg(AxisState xState, AxisState yState, AxisState zState);
        void getPowerManagement2Reg(AxisState& xState, AxisState& yState, AxisState& zState);

        // Setup
        /////////////////////////////////////////////
        void setRangeScaleFactor();

        // Misc
        /////////////////////////////////////////////
        /// \brief Converts a value from the register in units LSB to a useable reading in G
        double convertReadingRawToUseable(uint32_t reading) { return (double)(reading) / rangeScaleFactor_; }

        /// \brief Converts a value in G to a value for the register in units LSB
        uint32_t convertReadingUseableToRaw(double reading) { return (uint32_t)(reading * rangeScaleFactor_); }

        // Members
        /////////////////////////////////////////////
        uint8_t chipSelect_;
        uint32_t spiClockSpeed_;
        GyroscopeConfig config_;
        float rangeScaleFactor_;
    };
};
#endif
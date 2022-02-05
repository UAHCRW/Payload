#include "IAM_20380.hpp"

namespace IAM20380
{
    Gyroscope::Gyroscope(uint8_t chipSelectPin, uint32_t clock, GyroscopeConfig& config)
    {
        chipSelect_    = chipSelectPin;
        spiClockSpeed_ = clock;
        config_        = config;
    }

    // -----------------------------------------------------------------------------------------------------------------
    bool Gyroscope::begin()
    {
        if (!validateWhoAmIReg())
        {
            Logger::error("IAM 20380 Gyroscope failed to validate who am I register! ");
            return false;
        }

        // Set the sample rate
        setSampleRateDividerReg(0);
        setGyroConfigReg(config_.range, false);
        setUserControlRegister(config_.fifoEnabled, config_.i2cDisabled, false, false);
        setPowerManagement1Reg(false, config_.sleepEnabled, config_.inStandby, config_.tempDisabled);
        setPowerManagement2Reg(config_.xAxisState, config_.yAxisState, config_.zAxisState);

        return true;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint16_t Gyroscope::getRawTemperature()
    {
        uint16_t value{0};
        uint8_t buff[2]{0};
        readRegisters(Register::TEMP_OUT_H, buff, 2);
        value |= (buff[1] << 8);
        value |= buff[0];
        return value;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getState(GyroData& data)
    {
        data.xRaw    = getRawGyroX();
        data.yRaw    = getRawGyroY();
        data.zRaw    = getRawGyroZ();
        data.tempRaw = getRawTemperature();
        data.x       = convertReadingRawToUseable(data.xRaw);
        data.y       = convertReadingRawToUseable(data.yRaw);
        data.z       = convertReadingRawToUseable(data.zRaw);
        data.temp    = (double)(data.tempRaw / IAM_20380_TEMP_SENSITIVITY_FACTOR + 25.0);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getStateFromFifo(GyroData& data)
    {
        uint8_t buff[6]{0};
        readRegisters(Register::FIFO_R_W, buff, 6);
        for (int ii = 1; ii >= 0; ii--) { data.xRaw |= (buff[ii] << (ii > 0) ? (8 * ii) : 0); }
        for (int ii = 1; ii >= 0; ii--) { data.yRaw |= (buff[ii + 2] << (ii > 0) ? (8 * ii) : 0); }
        for (int ii = 1; ii >= 0; ii--) { data.zRaw |= (buff[ii + 4] << (ii > 0) ? (8 * ii) : 0); }

        data.x = convertReadingRawToUseable(data.xRaw);
        data.y = convertReadingRawToUseable(data.yRaw);
        data.z = convertReadingRawToUseable(data.zRaw);
    }

    void Gyroscope::getStateFromLastFifoSample(GyroData& data)
    {
        uint16_t numSamples = getNumBytesInFifo() / 6;
        uint8_t buff[6]{0};
        readRegisters(Register::FIFO_R_W, buff, 9);
        for (int ii = 0; ii < numSamples; ii++) { readRegisters(Register::FIFO_R_W, buff, 6); }
        for (int ii = 1; ii >= 0; ii--) { data.xRaw |= (buff[ii] << (ii > 0) ? (8 * ii) : 0); }
        for (int ii = 1; ii >= 0; ii--) { data.yRaw |= (buff[ii + 2] << (ii > 0) ? (8 * ii) : 0); }
        for (int ii = 1; ii >= 0; ii--) { data.zRaw |= (buff[ii + 4] << (ii > 0) ? (8 * ii) : 0); }

        data.x = convertReadingRawToUseable(data.xRaw);
        data.y = convertReadingRawToUseable(data.yRaw);
        data.z = convertReadingRawToUseable(data.zRaw);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setGyroBias(GyroData& bias)
    {
        uint8_t buff[2]{0};
        uint16_t x = convertReadingUseableToRaw(bias.x);
        uint16_t y = convertReadingUseableToRaw(bias.y);
        uint16_t z = convertReadingUseableToRaw(bias.z);

        // Write X bias
        buff[0] = x >> 8;
        buff[1] = x & 0xFF;
        writeRegisters(Register::XG_OFFS_USRH, buff, 2);

        buff[0] = y >> 8;
        buff[1] = x & 0xFF;
        writeRegisters(Register::YG_OFFS_USRH, buff, 2);

        buff[0] = z >> 8;
        buff[1] = z & 0xFF;
        writeRegisters(Register::ZG_OFFS_USRH, buff, 2);
    }
    // -----------------------------------------------------------------------------------------------------------------
    GyroData Gyroscope::getGyroBias()
    {
        GyroData bias;
        bias.x = readRawGyrometerReg(Register::XG_OFFS_USRH);
        bias.y = readRawGyrometerReg(Register::YG_OFFS_USRH);
        bias.z = readRawGyrometerReg(Register::ZG_OFFS_USRH);
        return bias;
    }

    // -----------------------------------------------------------------------------------------------------------------
    GyroscopeConfig Gyroscope::getGyroConfig()
    {
        GyroscopeConfig config;
        config.bias = getGyroBias();
        bool h1, h2; // holders for un needed values
        getUserControlRegister(config.fifoEnabled, config.i2cDisabled, h1, h2);
        getPowerManagement1Reg(h1, config.sleepEnabled, config.inStandby, config.tempDisabled);
        getPowerManagement2Reg(config.xAxisState, config.yAxisState, config.zAxisState);
        config.sampleRateDivider = getSampleRateDividerReg();
        getGyroConfigReg(config.range, h1);
        return config;
    }
    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::printGryoConfiguration()
    {
        auto boolalpha         = [](bool x) { return ((x) ? "true" : "false"); };
        auto axisPrint         = [](AxisState x) { return ((x == AxisState::ENABLED) ? "ENABLED" : "DISABLED"); };
        GyroscopeConfig config = getGyroConfig();
        Logger::notice("---------------------------------------------------------------------");
        Logger::notice("                IAM-20380 Gyrometer Configuration");
        Logger::notice("---------------------------------------------------------------------");
        Logger::notice(" * Axis States");
        Logger::notice(("\tX Axis: " + String(axisPrint(config.xAxisState))).c_str());
        Logger::notice(("\tY Axis: " + String(axisPrint(config.yAxisState))).c_str());
        Logger::notice(("\tZ Axis: " + String(axisPrint(config.zAxisState)) + "\n").c_str());

        Logger::notice(" * Axis Bias:");
        Logger::notice(("\tX Axis: " + String(config.bias.x)).c_str());
        Logger::notice(("\tY Axis: " + String(config.bias.y)).c_str());
        Logger::notice(("\tZ Axis: " + String(config.bias.z) + "\n").c_str());

        Logger::notice(" * Sensor Configuration");
        Logger::notice(("\tFIFO Enabled:                " + String(boolalpha(config.fifoEnabled))).c_str());
        Logger::notice(("\tI2C Disabled:                " + String(boolalpha(config.i2cDisabled))).c_str());
        Logger::notice(("\tSleep Enabled:               " + String(boolalpha(config.sleepEnabled))).c_str());
        Logger::notice(("\tIn Standby Mode:             " + String(boolalpha(config.inStandby))).c_str());
        Logger::notice(("\tTemperature Sensor Disabled: " + String(boolalpha(config.tempDisabled))).c_str());
        Logger::notice(("\tSample Rate Divider:         " + String(config.sampleRateDivider)).c_str());
        Logger::notice("---------------------------------------------------------------------\n");
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint16_t Gyroscope::getNumBytesInFifo()
    {
        uint8_t buff[2]{0};
        readRegisters(Register::FIFO_COUNTH, buff, 2);

        uint16_t val{0};
        val |= buff[1] << 8;
        val |= buff[0];
        return val;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Private
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Gyroscope::select()
    {
        SPI.beginTransaction(SPISettings(spiClockSpeed_, MSBFIRST, SPI_MODE0));
        digitalWrite(chipSelect_, LOW);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::deselect()
    {
        digitalWrite(chipSelect_, HIGH);
        SPI.endTransaction();
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint32_t Gyroscope::readRawGyrometerReg(Register reg)
    {
        uint8_t buff[3]{0};
        readRegisters(reg, buff, 3);
        uint32_t accel{0};
        for (int ii = 2; ii >= 0; ii--) { accel |= (buff[ii] << ((ii > 0) ? (8 * ii - 5) : 0)); }
        return accel;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes)
    {
        select();
        SPI.transfer(reg | 0x80);
        for (int ii = 0; ii < bytes; ii++) buffer[ii] = SPI.transfer(0x00);
        deselect();
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Gyroscope::readSingleRegister(uint8_t reg)
    {
        uint8_t buff[1];
        readRegisters(reg, buff, 1);
        return buff[0];
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes)
    {
        select();
        SPI.transfer(reg);
        for (int ii = 0; ii < bytes; ii++) SPI.transfer(buffer[ii]);
        deselect();
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::writeSingleRegister(uint8_t reg, uint8_t val)
    {
        uint8_t buff[1];
        buff[0] = val;
        writeRegisters(reg, buff, 1);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setGyroConfigReg(GyroRange range, bool dlpfBypass)
    {
        uint8_t val{0};
        val |= (uint8_t)(range) << 3;
        val |= (uint8_t)(dlpfBypass);
        writeSingleRegister(Register::GYRO_CONFIG, val);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getGyroConfigReg(GyroRange& range, bool& dlpfBypass)
    {
        uint8_t val = readSingleRegister(Register::GYRO_CONFIG);
        range       = (GyroRange)((val >> 3) & 0x3);
        dlpfBypass  = (bool)(val & 0x3);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setUserControlRegister(bool fifoEnable, bool i2cDisable, bool fifoReset, bool sigCondReset)
    {
        uint8_t val{0};
        val |= ((uint8_t)(fifoEnable) << 6);
        val |= ((uint8_t)(i2cDisable) << 4);
        val |= ((uint8_t)(fifoReset) << 2);
        val |= (uint8_t)(sigCondReset);
        writeSingleRegister(Register::USER_CTRL, val);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getUserControlRegister(bool& fifoEnable, bool& i2cDisable, bool& fifoReset, bool& sigCondReset)
    {
        uint8_t val  = readSingleRegister(Register::USER_CTRL);
        fifoEnable   = (bool)((val >> 6) & 0x01);
        i2cDisable   = (bool)((val >> 4) & 0x01);
        fifoReset    = (bool)((val >> 2) & 0x01);
        sigCondReset = (bool)(val & 0x01);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setPowerManagement1Reg(bool reset, bool sleep, bool standby, bool tempDisable)
    {
        uint8_t val{0};
        val |= (uint8_t)(reset) << 7;
        val |= (uint8_t)(sleep) << 6;
        val |= (uint8_t)(standby) << 4;
        val |= (uint8_t)(tempDisable) << 3;
        writeSingleRegister(Register::PWR_MGMT_1, val);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getPowerManagement1Reg(bool& reset, bool& sleep, bool& standby, bool& tempDisable)
    {
        uint8_t val = readSingleRegister(Register::PWR_MGMT_1);
        reset       = (bool)(val >> 7);
        sleep       = (bool)(val >> 6);
        standby     = (bool)(val >> 4);
        tempDisable = (bool)(val >> 3);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setPowerManagement2Reg(AxisState xState, AxisState yState, AxisState zState)
    {
        uint8_t val{0};
        val |= (uint8_t)(xState) << 2;
        val |= (uint8_t)(yState) << 1;
        val |= (uint8_t)(zState);
        writeSingleRegister(Register::PWR_MGMT_2, val);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::getPowerManagement2Reg(AxisState& xState, AxisState& yState, AxisState& zState)
    {
        uint8_t val = readSingleRegister(Register::PWR_MGMT_2);
        xState      = (AxisState)((val >> 2) & 0x01);
        yState      = (AxisState)((val >> 1) & 0x01);
        zState      = (AxisState)(val & 0x01);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Gyroscope::setRangeScaleFactor()
    {
        switch (config_.range)
        {
            case GyroRange::DPS_250: rangeScaleFactor_ = IAM_20380_RANGE_SCALE_FACTOR_500_DPS; break;
            case GyroRange::DPS_500: rangeScaleFactor_ = IAM_20380_RANGE_SCALE_FACTOR_500_DPS; break;
            case GyroRange::DPS_1000: rangeScaleFactor_ = IAM_20380_RANGE_SCALE_FACTOR_1000_DPS; break;
            case GyroRange::DPS_2000: rangeScaleFactor_ = IAM_20380_RANGE_SCALE_FACTOR_2000_DPS; break;
            default: rangeScaleFactor_ = 0;
        }
    }

};
#include "ADXL357_Accelerometer.hpp"

namespace ADXL357
{

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Public
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Accelerometer::Accelerometer(uint8_t chipSelectPin, uint8_t clock, AccelerometerConfig& config)
        : chipSelect_(chipSelectPin), spiClockSpeed_(clock), config_(config)
    {
    }

    // -----------------------------------------------------------------------------------------------------------------
    bool Accelerometer::begin()
    {
        // Check the status of the first three registers and make sure that the values are correct.

        // These registers never change. If these values dont match something is wrong
        if ((getAnalogDevicesID() != ANALOG_DEV_ID) || (getAnalogDevicesMemsID() != DEV_MEMS_ID) ||
            (getDeviceId() != DEVICE_ID))
        {
            Logger::error("One of all of the Analog Device ID, Analog MEMS ID, or Device ID didn't match ICD for the "
                          "ADXL357 Accelerometer");
            return false;
        }

        setI2CInterruptRangeReg(config_.i2cSpeed, config_.interruptPolarity, config_.sensorRange);
        setFilterSettings(config_.odrLpfSetting, config_.hpfSetting);
        setPwrControlReg(false, false, false);
        setFifoSamples(MAX_FIFO_SAMPLES);

        Logger::notice("ADXL357 has been turned on for measurement reporting");

        return true;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint16_t Accelerometer::getRawTemperature()
    {
        uint8_t buff[2]{0};
        readRegisters(REGISTER::TEMP2, buff, 2);
        uint16_t temp{0};
        for (int ii = 1; ii >= 0; ii++) { temp |= (buff[ii] << ((ii > 0) ? (8 * ii - 5) : 0)); }
        return temp;
    }

    // -----------------------------------------------------------------------------------------------------------------
    AccelerometerData Accelerometer::getState()
    {
        AccelerometerData data;
        data.x    = getAccelX();
        data.y    = getAccelY();
        data.z    = getAccelZ();
        data.temp = getTemperature();
        return data;
    }

    // -----------------------------------------------------------------------------------------------------------------
    AccelerometerData Accelerometer::getState(AccelerometerRawData& rawData)
    {
        AccelerometerData data;
        data.x    = rawData.x;
        data.y    = rawData.y;
        data.z    = rawData.z;
        data.temp = rawData.temp;
        return data;
    }

    // -----------------------------------------------------------------------------------------------------------------
    AccelerometerRawData Accelerometer::getRawState()
    {
        AccelerometerRawData data;
        data.x    = getRawAccelX();
        data.y    = getRawAccelY();
        data.z    = getRawAccelZ();
        data.temp = getRawTemperature();
        return data;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::setAccelerometerBias(AccelerometerData& data)
    {
        uint16_t x = (uint16_t)(data.x * 1e-6 / rangeScaleFactor_);
        uint16_t y = (uint16_t)(data.y * 1e-6 / rangeScaleFactor_);
        uint16_t z = (uint16_t)(data.z * 1e-6 / rangeScaleFactor_);
        uint8_t buff[2]{0};
        buff[0] = (x >> 8) & 0xFF;
        buff[1] = x & 0xFF;
        writeRegisters(REGISTER::OFFSET_X_H, buff, 2);
        buff[0] = (y >> 8) & 0xFF;
        buff[1] = y & 0xFF;
        writeRegisters(REGISTER::OFFSET_Z_H, buff, 2);
        buff[0] = (z >> 8) & 0xFF;
        buff[1] = z & 0xFF;
        writeRegisters(REGISTER::OFFSET_Y_H, buff, 2);
    }

    // -----------------------------------------------------------------------------------------------------------------
    AccelerometerData Accelerometer::getAccelerometerBias()
    {
        AccelerometerData data;
        uint8_t buff[2]{0};
        uint16_t val{0};
        readRegisters(REGISTER::OFFSET_X_H, buff, 2);
        for (int ii = 1; ii >= 0; ii--) { val |= buff[ii] << ((ii > 0) ? 8 : 0); }
        data.x = converReadingRawToUseable(val);

        val = 0;
        readRegisters(REGISTER::OFFSET_Y_H, buff, 2);
        for (int ii = 1; ii >= 0; ii--) { val |= buff[ii] << ((ii > 0) ? 8 : 0); }
        data.y = converReadingRawToUseable(val);

        val = 0;
        readRegisters(REGISTER::OFFSET_Z_H, buff, 2);
        for (int ii = 1; ii >= 0; ii--) { val |= buff[ii] << ((ii > 0) ? 8 : 0); }
        data.z = converReadingRawToUseable(val);
        return data;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getAnalogDevicesID()
    {
        uint8_t val{0};
        readSingleRegister(REGISTER::DEV_ID_AD, val);
        return val;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getAnalogDevicesMemsID()
    {
        uint8_t val{0};
        readSingleRegister(REGISTER::DEV_ID_MEM, val);
        return val;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getDeviceId()
    {
        uint8_t val{0};
        readSingleRegister(REGISTER::DEV_ID, val);
        return val;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getProductRevisionID()
    {
        uint8_t val{0};
        readSingleRegister(REGISTER::REV_ID, val);
        return val;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::printSensorConfiguration()
    {
        AccelerometerConfig config = getAccelerometerConfig();
        Logger::notice("---------------------------------------------------------------------");
        Logger::notice("                ADXL357 Accelerometer Configuration");
        Logger::notice("---------------------------------------------------------------------");
        Logger::notice(" * Data Rate [Hz] Low Pass Filter [Hz]:");

        switch (config.odrLpfSetting)
        {
            case ODR_LPF::ODR_4000_LP_1000: Logger::notice("\t4000 Hz and 1000 Hz\n"); break;
            case ODR_LPF::ODR_2000_LP_500: Logger::notice("\t2000 Hz and 500 Hz\n"); break;
            case ODR_LPF::ODR_1000_LP_250: Logger::notice("\t1000 Hz and 250 Hz\n"); break;
            case ODR_LPF::ODR_500_LP_125: Logger::notice("\t500 Hz and 125 Hz\n"); break;
            case ODR_LPF::ODR_250_LP_62_5: Logger::notice("\t250 Hz and 62.5 Hz\n"); break;
            case ODR_LPF::ODR_125_LP_31_25: Logger::notice("\t125 Hz and 31.25 Hz\n"); break;
            case ODR_LPF::ODR_62_5_LP_15_625: Logger::notice("\t62.5 Hz and 15.625 Hz\n"); break;
            case ODR_LPF::ODR_31_25_LP_7_813: Logger::notice("\t31.25 Hz and 7.813 Hz\n"); break;
            case ODR_LPF::ODR_15_625_LP_3_906: Logger::notice("\t15.625 Hz and 3.906 Hz\n"); break;
            case ODR_LPF::ODR_7_813_LP_1_953: Logger::notice("\t7.813 Hz and 1.953 Hz\n"); break;
            case ODR_LPF::ODR_3_906_LP_0_977: Logger::notice("\t3.906 Hz and 0.977 Hz\n"); break;
            default: Logger::warning("\tCannot be determined."); break;
        }

        Logger::notice(" * High Pass Filter [Hz]");
        switch (config.hpfSetting)
        {
            case HIGH_PASS_FILTER::HP_24_7: Logger::notice("\t24.7 x 10^-4 X ODR\n"); break;
            case HIGH_PASS_FILTER::HP_6_2084: Logger::notice("\t6.2084 x 10^-4 X ODR\n"); break;
            case HIGH_PASS_FILTER::HP_1_5545: Logger::notice("\t1.5545 x 10^-4 X ODR\n"); break;
            case HIGH_PASS_FILTER::HP_3862: Logger::notice("\t0.3862 x 10^-4 X ODR\n"); break;
            case HIGH_PASS_FILTER::HP_0954: Logger::notice("\t0.0954 x 10^-4 X ODR\n"); break;
            case HIGH_PASS_FILTER::HP_0238: Logger::notice("\t0.0238 x 10^-4 X ODR\n"); break;
            default: Logger::warning("\tCannot be determined."); break;
        }

        Logger::notice(" * I2C Speed");
        switch (config.i2cSpeed)
        {
            case I2CSpeed::HIGH_SPEED: Logger::notice("\tHigh Speed\n"); break;
            case I2CSpeed::LOW_SPEED: Logger::notice("\tLow Speed\n"); break;
            default: Logger::warning("\tCannot be determined\n"); break;
        }

        Logger::notice(" * Interrupt Polarity");
        switch (config.interruptPolarity)
        {
            case InterruptPolarity::ACTIVE_HIGH: Logger::notice("\tActive High\n"); break;
            case InterruptPolarity::ACTIVE_LOW: Logger::notice("\tActive Low\n"); break;
            default: Logger::warning("\tCannot be determined\n"); break;
        }

        Logger::notice(" * Sensor Range");
        switch (config.sensorRange)
        {
            case AccelRange::TEN: Logger::notice("\t10 g\n"); break;
            case AccelRange::TWENTY: Logger::notice("\t20 g\n"); break;
            case AccelRange::FORTY: Logger::notice("\t40 g\n"); break;
            default: Logger::warning("\tCannot be determined\n"); break;
        }

        Logger::notice(" * Initial Accelerometer Bias");
        Logger::notice(("\t X [m/s^2]: " + String(config_.sensorBias.x)).c_str());
        Logger::notice(("\t Y [m/s^2]: " + String(config_.sensorBias.y)).c_str());
        Logger::notice(("\t Z [m/s^2]: " + String(config_.sensorBias.z) + "\n").c_str());
        Logger::notice("---------------------------------------------------------------------\n");
    }

    // -----------------------------------------------------------------------------------------------------------------
    AccelerometerConfig Accelerometer::getAccelerometerConfig()
    {
        AccelerometerConfig config;
        getI2CInterruptRangeReg(config.i2cSpeed, config.interruptPolarity, config.sensorRange);
        config.sensorBias = getAccelerometerBias();
        getFilterSettings(config.odrLpfSetting, config.hpfSetting);
        return config;
    }

    // -----------------------------------------------------------------------------------------------------------------
    bool Accelerometer::isDataReady()
    {
        bool nvmBusy{false}, activity{false}, fifoOverrun{true}, fifoFull{true}, dataRdy{false};
        getStatus(nvmBusy, activity, fifoOverrun, fifoFull, dataRdy);
        return dataRdy;
    }

    // -----------------------------------------------------------------------------------------------------------------
    bool Accelerometer::isFifoFull()
    {
        bool nvmBusy{false}, activity{false}, fifoOverrun{true}, fifoFull{true}, dataRdy{false};
        getStatus(nvmBusy, activity, fifoOverrun, fifoFull, dataRdy);
        return fifoFull;
    }

    // -----------------------------------------------------------------------------------------------------------------
    bool Accelerometer::hasFifoOverrun()
    {
        bool nvmBusy{false}, activity{false}, fifoOverrun{true}, fifoFull{true}, dataRdy{false};
        getStatus(nvmBusy, activity, fifoOverrun, fifoFull, dataRdy);
        return fifoOverrun;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::numValidFifoSamples()
    {
        uint8_t samps{0};
        readSingleRegister(REGISTER::FIFO_SAMPLES, samps);
        return samps;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Private
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Accelerometer::select()
    {
        SPI.beginTransaction(SPISettings(spiClockSpeed_, MSBFIRST, SPI_MODE0));
        digitalWrite(chipSelect_, LOW);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::deselect()
    {
        digitalWrite(chipSelect_, HIGH);
        SPI.endTransaction();
    }

    // -----------------------------------------------------------------------------------------------------------------
    double Accelerometer::readAccelerationReg(REGISTER reg)
    {
        uint32_t rawRegVal = readRawAccelerationReg(reg);
        return converReadingRawToUseable(rawRegVal);
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint32_t Accelerometer::readRawAccelerationReg(REGISTER reg)
    {
        uint8_t buff[3]{0};
        readRegisters(reg, buff, 3);
        uint32_t accel{0};
        for (int ii = 2; ii >= 0; ii--) { accel |= (buff[ii] << ((ii > 0) ? (8 * ii - 5) : 0)); }
        return accel;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes)
    {
        select();
        SPI.transfer((reg << 1) | 0x1);
        for (int ii = 0; ii < bytes; ii++) buffer[ii] = SPI.transfer(0x00);
        deselect();
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::readSingleRegister(uint8_t reg, uint8_t& data)
    {
        uint8_t buff[1];
        readRegisters(reg, buff, 1);
        data = buff[0];
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t bytes)
    {
        select();
        SPI.transfer((reg << 1));
        for (int ii = 0; ii < bytes; ii++) SPI.transfer(buffer[ii]);
        deselect();
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::writeSingleRegister(uint8_t reg, uint8_t val)
    {
        uint8_t buff[1];
        buff[0] = val;
        writeRegisters(reg, buff, 1);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::setRangeScaleFactor()
    {
        rangeScaleFactor_ = 0.0;

        if (config_.sensorRange == AccelRange::TEN)
            rangeScaleFactor_ = RANGE_SCALE_FACTOR_10;
        else if (config_.sensorRange == AccelRange::TWENTY)
            rangeScaleFactor_ = RANGE_SCALE_FACTOR_20;
        else if (config_.sensorRange == AccelRange::FORTY)
            rangeScaleFactor_ = RANGE_SCALE_FACTOR_40;
    }

    // -----------------------------------------------------------------------------------------------------------------
    double Accelerometer::converReadingRawToUseable(uint32_t reading)
    {
        // Handle two's compliment
        int32_t newVal = ((int32_t)(reading << 4)) >> 4;
        return ((double)(newVal)) * rangeScaleFactor_ / 1e-6;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint32_t Accelerometer::converReadingUseableToRaw(double reading)
    {
        return ((uint32_t)(reading / rangeScaleFactor_ * 1e-6));
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::setPwrControlReg(bool drdyOn, bool tempOn, bool standby)
    {
        uint8_t regVal{0};
        regVal |= ((uint8_t)(drdyOn) << 2);
        regVal |= ((uint8_t)(tempOn) << 1);
        regVal |= ((uint8_t)(standby));
        writeSingleRegister(REGISTER::POWER_CTL, regVal);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::getPwrControlReg(bool& drdyOn, bool& tempOn, bool& standby)
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::POWER_CTL, regVal);
        drdyOn  = (bool)((regVal >> 2) & 0x01);
        tempOn  = (bool)((regVal >> 1) & 0x01);
        standby = (bool)(regVal & 0x01);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::setI2CInterruptRangeReg(I2CSpeed i2cSpeed, InterruptPolarity polarity, AccelRange range)
    {
        uint8_t regVal{0};
        regVal |= (uint8_t)(i2cSpeed) << 7;
        regVal |= (uint8_t)(polarity) << 6;
        regVal |= (uint8_t)(range);
        writeSingleRegister(REGISTER::ACCEL_RANGE, regVal);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::getI2CInterruptRangeReg(I2CSpeed& i2cSpeed, InterruptPolarity& polarity, AccelRange& range)
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::ACCEL_RANGE, regVal);
        i2cSpeed = (I2CSpeed)(regVal >> 7);
        polarity = (InterruptPolarity)((regVal >> 6) & 0x01);
        range    = (AccelRange)(regVal & 0x03);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::setFilterSettings(ODR_LPF odrLpf, HIGH_PASS_FILTER hpf)
    {
        uint8_t regVal{0};
        regVal |= ((uint8_t)(odrLpf)) & 0xF;
        regVal |= (uint8_t)(hpf) << 4;
        writeSingleRegister(REGISTER::FILTER, regVal);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::getFilterSettings(ODR_LPF& odrLpf, HIGH_PASS_FILTER& hpf)
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::FILTER, regVal);
        odrLpf = (ODR_LPF)(regVal & 0xF);
        hpf    = (HIGH_PASS_FILTER)(regVal >> 4);
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getFifoSamples()
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::FIFO_SAMPLES, regVal);
        return regVal;
    }

    // -----------------------------------------------------------------------------------------------------------------
    uint8_t Accelerometer::getFifoValidSamples()
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::FIFO_ENTRIES, regVal);
        return regVal;
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::getFifoData(double& x, double& y, double& z)
    {
        uint8_t buff[9]{0};
        readRegisters(REGISTER::FIFO_DATA, buff, 9);

        // Check to make sure the first result is along the x-axis
        if ((buff[2] & 0x1) != 1) Logger::error("FIFO read found first entry not to correspond to the X-Axis!");

        if (((buff[2] >> 1) & 0x1) != 0) Logger::error("FIFO read found that read was done on an empty FIFO Buffer!");

        uint32_t xRaw = 0, yRaw = 0, zRaw = 0;
        for (int ii = 2; ii >= 0; ii--) { xRaw |= (buff[ii] << (ii > 0) ? (8 * ii - 4) : 0); }
        for (int ii = 2; ii >= 0; ii--) { yRaw |= (buff[ii + 3] << (ii > 0) ? (8 * ii - 4) : 0); }
        for (int ii = 2; ii >= 0; ii--) { zRaw |= (buff[ii + 6] << (ii > 0) ? (8 * ii - 4) : 0); }

        x = converReadingRawToUseable(xRaw);
        y = converReadingRawToUseable(yRaw);
        z = converReadingRawToUseable(zRaw);
    }

    // -----------------------------------------------------------------------------------------------------------------
    void Accelerometer::getStatus(bool& nvmBusy, bool& activity, bool& fifoOverrun, bool& fifoFull, bool& dataRdy)
    {
        uint8_t regVal{0};
        readSingleRegister(REGISTER::STATUS, regVal);
        nvmBusy     = (bool)((regVal >> 4) & 0x01);
        activity    = (bool)((regVal >> 3) & 0x01);
        fifoOverrun = (bool)((regVal >> 2) & 0x01);
        fifoFull    = (bool)((regVal >> 1) & 0x01);
        dataRdy     = (bool)(regVal & 0x01);
    }
};
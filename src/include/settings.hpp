#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include "MPU6050.h"
#include "logger.hpp"
// #include "include/MPU6050.h"
// #include "include/logger.hpp"
#include <stdint.h>

class Settings final
{
    public:
    Settings();
    ~Settings(){};

    bool initializeIMU(MPU6050* sensor);
    bool initializeCrwAccelerometer() { return false; }
    bool initiailzeCrwGyroscope() { return false; }
    bool initializeMagnetometer() { return false; }
    bool initializePressureSensor() { return false; }

    char* getTrajectoryFilename() { return &trajectoryFileName_[0]; }
    char* getLoggingFilename() { return &loggingFileName_[0]; }
    Logger::Level getLoggingLevel() { return loggingLevel_; }
    uint32_t getBaudRate() { return baudRate_; }
    mpu6050_range_t getMpuAccelRange() { return mpuAccelerometerRange_; }
    mpu6050_dps_t getMpuGyroRange() { return mpuGyrometerRange_; }
    bool isMpu6050Initialized() { return mpu6050Initialized_; }

    private:
    void convertIntToChar(const char* message, char* buf, uint64_t val);
    char trajectoryFileName_[60];
    char loggingFileName_[60];
    Logger::Level loggingLevel_;
    uint32_t baudRate_;

    // MPU 6050
    mpu6050_range_t mpuAccelerometerRange_;
    mpu6050_dps_t mpuGyrometerRange_;
    bool mpu6050Initialized_;
};

#endif
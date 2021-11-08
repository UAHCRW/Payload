#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include "Adafruit_LIS3MDL.h"
#include "Arduino.h"
#include "MPU6050.h"
#include "logger.hpp"
#include <stdint.h>

class Settings final
{
    public:
    Settings();
    ~Settings(){};

    bool initializeIMU(MPU6050* sensor);
    bool initializeCrwAccelerometer() { return false; }
    bool initiailzeCrwGyroscope() { return false; }
    bool initializeMagnetometer(Adafruit_LIS3MDL* sensor, uint8_t clockPin);
    bool initializePressureSensor() { return false; }

    void loadConfiguration(const char* filename){};

    void printCrwPayloadSettings();
    String getTrajectoryFilename() { return trajectoryFileName_; }
    String getLoggingFilename() { return loggingFileName_; }
    Logger::Level getLoggingLevel() { return loggingLevel_; }
    uint32_t getBaudRate() { return baudRate_; }
    bool isMpu6050Initialized() { return mpu6050Initialized_; }
    float getTimeInterval() { return timeInterval_; }

    private:
    // General
    String trajectoryFileName_;
    String loggingFileName_;
    Logger::Level loggingLevel_;
    uint32_t baudRate_;
    float samplingFreq_;
    float timeInterval_;

    // MPU 6050
    mpu6050_range_t mpuAccelerometerRange_;
    mpu6050_dps_t mpuGyrometerRange_;
    bool mpu6050Initialized_;

    // LIS3MDL Magnetometer
    bool magnetometerInitialized_;
};

#endif
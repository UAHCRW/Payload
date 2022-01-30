#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include "ADXL357_Accelerometer.hpp"
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

    /// \brief Scans network for available I2C addresses
    void scanI2CNetwork();

    /// \brief Initializes an IMU based on settings and prints those settings out
    /// \param sensor A pointer to a MPU6050 sensor object
    bool initializeIMU(MPU6050* sensor);

    /// \brief Initializes and prints settings for the ADXL357 -- not implemented
    bool initializeCrwAccelerometer(ADXL357::Accelerometer* accelerometer);

    /// \brief Initializes the Gyroscope and prints settings
    bool initiailzeCrwGyroscope() { return false; }

    /// \brief INitializes magnetometer and prints settings
    /// \param sensor A pointer to a magnetometer sensor object
    /// \param pin Pin number for the pin being used as the chip select pin
    bool initializeMagnetometer(Adafruit_LIS3MDL* sensor, uint8_t pin);

    /// \brief Initializes the pressure sensor
    bool initializePressureSensor() { return false; }

    /// \brief Initializes settings based upon a configuration file
    /// \param filename a pointer to a char array containing the filename
    void loadConfiguration(const char* filename){};

    /// \brief Prints the Payload settings
    void printCrwPayloadSettings();

    /// \brief Confirms whether the IMU is connected
    /// \returns true if connected otherwise false
    bool isMpu6050Initialized() { return mpu6050Initialized_; }

    /// \brief Confirms whether the IMU is connected
    /// \returns true if connected otherwise false
    bool isMagnetometerInitialized() { return magnetometerInitialized_; }

    /// \brief The time interval between sensor samples in seconds (1/sampling freq)
    /// \returns time interval in seconds
    float getTimeIntervalSeconds() { return timeInterval_; }

    /// \brief Gets the time interval between sensor samples in milliseconds (1/sampling freq) * 1000
    /// \returns time interval in milliseconds
    float getTimeIntervalMilliSeconds() { return (timeInterval_ * 1000); }

    /// \brief Returns the name of the trajectory file
    String getTrajectoryFilename() { return trajectoryFileName_; }

    /// \brief Returns the name of the logging file
    String getLoggingFilename() { return loggingFileName_; }

    /// \brief Returns the baud rate
    uint32_t getBaudRate() { return baudRate_; }

    /// \brief Returns the base logging level, any messages above this level are ignored
    /// level defaults to the highest possible
    Logger::Level getLoggingLevel() { return loggingLevel_; }

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
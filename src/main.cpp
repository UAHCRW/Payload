/*
Retrieves sensor data and writes to a csv file on the sd card
Sensors:
  Magnetometer x1
  Accelerometer x3
  Gyroscope x3

Author: Charger Rocket Works 2021/2022 Team
Date: Fall 2021
*/
#include "ADXL356_Accelerometer.hpp"
#include "Adafruit_LIS3MDL.h"
#include "Adafruit_Sensor.h"
#include "Arduino.h"
#include "Logger.hpp"
#include "MPU6050.h"
#include "SPI.h"
#include "SdFat.h"
#include "settings.hpp"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// SD Card Initialization
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// Use built-in SD for SPI modes on Teensy 3.5/3.6.
// Teensy 4.0 use first SPI port.
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#    define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#    define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else // HAS_SDIO_CLASS
#    define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif // HAS_SDIO_CLASS

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Program Configuration & Definitions (Pins, Sensors, Constants, Functions)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define RAW_MAGNETOMETER_CHIP_SELECT 12

Adafruit_LIS3MDL magnetomer_;
sensors_event_t magEvent_;

MPU6050 mpu_;
Settings settings_;
float readTime_ = 0;
float roll_     = 0;
float pitch_    = 0;
float yaw_      = 0;

SdFs sd;
FsFile trajectoryFile_;
FsFile loggingFile_;

Vector mpuRawAccel_;
Vector mpuNormAccel_;
Vector mpuRawGyro_;
Vector mpuNormGyro_;
float mpuTemp_;
Activites mpu6050Activities_;

// Functions
/// \brief Opens a file system
/// \param file a SD card file object
/// \param filename pointer to char array containing the filename
void openFile(FsFile& file, const char* filename);

/// \brief a callback that is used for logging, all calls to the logger will call this function internally
/// This functions sends the message out over serial and saves to a file if the file is open
/// \param level The logging level
/// \param module The module (class/function) that made the call to the logger (sometimes not used)
/// \param message The message to be logged
void crwLogger(Logger::Level level, const char* module, const char* message);

/// \brief Writes data to a csv file
/// \param file Sd Card file object
/// \param data data to write to the file
void writeDataToCsv(FsFile& file, Vector& data, bool endLine = false);
void writeDataToCsv(FsFile& file, int16_t& data, bool endLine = false);
void writeDataToCsv(FsFile& file, float& data, bool endLine = false);
void writeDataToCsv(FsFile& file, double& data, bool endLine = false);
void writeDataToCsv(FsFile& file, Activites& data, bool endLine = false);
void writeDataToCsv(FsFile& file, sensors_event_t& data, bool endLine = false);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Setup()
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()
{
    Serial.begin(settings_.getBaudRate());

    Wire.setSCL(19);
    Wire.setSDA(18);
    Wire.begin();

    Logger::setOutputFunction(crwLogger);
    Logger::setLogLevel(settings_.getLoggingLevel());

    pinMode(LED_BUILTIN, OUTPUT);

#ifdef USE_SD_CARD
    // Initialize the SD.
    Logger::notice("Mounting SD Card");
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
        return;
    }

    Logger::notice("SD Card Succesfully mounted");

    openFile(trajectoryFile_, settings_.getTrajectoryFilename().c_str());
    openFile(loggingFile_, settings_.getLoggingFilename().c_str());
#endif

    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("          Dead Reckoning Navigation System (DRNS) Startup");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("");
    settings_.scanI2CNetwork();
    settings_.printCrwPayloadSettings();

    settings_.initializeIMU(&mpu_);

    settings_.initializeMagnetometer(&magnetomer_, RAW_MAGNETOMETER_CHIP_SELECT);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
    readTime_ = millis();

    // Get values from the mpu IMU
    if (settings_.isMpu6050Initialized())
    {
        mpuRawAccel_ = mpu_.readRawAccel();
        // mpuNormAccel_ = mpu_.readScaledAccel();
        mpuNormAccel_      = mpu_.readScaledAccel();
        mpuRawGyro_        = mpu_.readRawGyro();
        mpuNormGyro_       = mpu_.readNormalizeGyro();
        mpu6050Activities_ = mpu_.readActivites();
        mpuTemp_           = mpu_.readTemperature();
    }

    // Get the values from the magnetometer
    if (settings_.isMagnetometerInitialized())
    {
        magnetomer_.read();
        magnetomer_.getEvent(&magEvent_);
    }

    roll_  = roll_ + mpuNormGyro_.XAxis * settings_.getTimeInterval();
    pitch_ = pitch_ + mpuNormGyro_.YAxis * settings_.getTimeInterval();
    yaw_   = yaw_ + mpuNormGyro_.ZAxis * settings_.getTimeInterval();

#ifdef USE_SD_CARD
    if (settings_.isMpu6050Initialized())
    {
        writeDataToCsv(trajectoryFile_, readTime_);
        writeDataToCsv(trajectoryFile_, mpuRawAccel_);
        writeDataToCsv(trajectoryFile_, mpuNormAccel_);
        writeDataToCsv(trajectoryFile_, mpuRawGyro_);
        writeDataToCsv(trajectoryFile_, mpuNormGyro_);

        writeDataToCsv(trajectoryFile_, roll_);
        writeDataToCsv(trajectoryFile_, pitch_);
        writeDataToCsv(trajectoryFile_, yaw_);
        writeDataToCsv(trajectoryFile_, mpu6050Activities_);

        writeDataToCsv(trajectoryFile_, mpuTemp_, !settings_.isMagnetometerInitialized());
    }

    if (settings_.isMagnetometerInitialized())
    {
        writeDataToCsv(trajectoryFile_, magnetomer_.x);
        writeDataToCsv(trajectoryFile_, magnetomer_.y);
        writeDataToCsv(trajectoryFile_, magnetomer_.z);
        writeDataToCsv(trajectoryFile_, magEvent_, true);
    }
    trajectoryFile_.flush();
#endif
    Serial.print(readTime_);
    Serial.print(",");
    Serial.print(mpuNormAccel_.XAxis);
    Serial.print(",");
    Serial.print(mpuNormAccel_.YAxis);
    Serial.print(",");
    Serial.print(mpuNormAccel_.ZAxis);
    Serial.print(",");
    Serial.print(mpuNormGyro_.XAxis);
    Serial.print(",");
    Serial.print(mpuNormGyro_.YAxis);
    Serial.print(",");
    Serial.print(mpuNormGyro_.ZAxis);
    Serial.print(",");
    // Serial.print(magnetomer_.x);
    // Serial.print(",");
    // Serial.print(magnetomer_.y);
    // Serial.print(",");
    // Serial.println(magnetomer_.z);
    Serial.print(magEvent_.magnetic.x);
    Serial.print(",");
    Serial.print(magEvent_.magnetic.y);
    Serial.print(",");
    Serial.println(magEvent_.magnetic.z);

    // We spent some time writing data and reading sensors so factor that in before next sample
    delay((settings_.getTimeInterval() * 1000) - (millis() - readTime_));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void openFile(FsFile& file, const char* filename)
{
    int fileAppend = 0; // Add a value to begining of file if one already exists
    String newFilename(filename);
    while (sd.exists(newFilename.c_str()))
    {
        fileAppend++;
        String append(String(fileAppend) + "_" + String(filename));
        newFilename = append;
    }

    if (!file.open(newFilename.c_str(), O_CREAT | O_WRITE))
    {
        String msg("File failed to open [" + String(filename) + "]");
        Logger::error(msg.c_str());
    }
    else
    {
        String msg("File [" + String(filename) + "] succesfully opened");
        Logger::notice(msg.c_str());
    }
}

void crwLogger(Logger::Level level, const char* module, const char* message)
{
    if (loggingFile_.isOpen())
    {
        loggingFile_.print(Logger::asString(level));
        loggingFile_.print(module);
        loggingFile_.println(message);
        loggingFile_.flush();
    }

    Serial.print(Logger::asString(level));
    Serial.print(module);
    Serial.println(message);
}

void writeDataToCsv(FsFile& file, Vector& data, bool endLine)
{
    file.print(data.XAxis);
    file.print(",");
    file.print(data.YAxis);
    file.print(",");
    if (endLine)
        file.println(data.ZAxis);
    else
    {
        file.print(data.ZAxis);
        file.print(",");
    }
}
void writeDataToCsv(FsFile& file, int16_t& data, bool endLine)
{
    if (endLine)
        file.println(data);
    else
    {
        file.print(data);
        file.print(",");
    }
}
void writeDataToCsv(FsFile& file, float& data, bool endLine)
{
    if (endLine)
        file.println(data);
    else
    {
        file.print(data);
        file.print(",");
    }
}
void writeDataToCsv(FsFile& file, double& data, bool endLine)
{
    if (endLine)
        file.println(data);
    else
    {
        file.print(data);
        file.print(",");
    }
}
void writeDataToCsv(FsFile& file, Activites& data, bool endLine)
{
    file.print(data.isActivity);
    file.print(F(","));
    file.print(data.isInactivity);
    file.print(F(","));
    file.print(data.isPosActivityOnX);
    file.print(F(","));
    file.print(data.isNegActivityOnX);
    file.print(F(","));
    file.print(data.isPosActivityOnY);
    file.print(F(","));
    file.print(data.isNegActivityOnY);
    file.print(F(","));
    file.print(data.isPosActivityOnZ);
    file.print(F(","));
    file.print(data.isNegActivityOnZ);
    file.print(F(","));
    file.print(data.isDataReady);
    file.print(F(","));
    file.print(data.isOverflow);
    file.print(F(","));
    if (endLine)
        file.println(data.isFreeFall);
    else
    {
        file.print(data.isFreeFall);
        file.print(F(","));
    }
}
void writeDataToCsv(FsFile& file, sensors_event_t& data, bool endLine)
{
    file.print(data.magnetic.x);
    file.print(F(","));
    file.print(data.magnetic.y);
    file.print(F(","));

    if (endLine)
        file.println(data.magnetic.z);
    else
    {
        file.print(data.magnetic.z);
        file.print(F(","));
    }
}

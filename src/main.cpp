/*
Retrieves sensor data and writes to a csv file on the sd card
Sensors:
  Magnetometer x1
  Accelerometer x3
  Gyroscope x3

Author: Charger Rocket Works 2021/2022 Team
Date: Fall 2021
*/
#include "Arduino.h"
#include "SPI.h"
#include "SdFat.h"
#include "include/ADXL356_Accelerometer.hpp"
#include "include/Logger.hpp"
#include "include/MPU6050.h"
#include "include/settings.hpp"

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
#define PIN_D7 12

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
void openFile(FsFile& file, const char* filename);
void crwLogger(Logger::Level level, const char* module, const char* message);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Setup()
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()
{
    Logger::setOutputFunction(crwLogger);
    Logger::setLogLevel(settings_.getLoggingLevel());

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(settings_.getBaudRate());

    Logger::notice("Mounting SD Card");

    // Initialize the SD.
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
        return;
    }

    Logger::notice("SD Card Succesfully mounted");

    openFile(trajectoryFile_, settings_.getTrajectoryFilename().c_str());
    openFile(loggingFile_, settings_.getLoggingFilename().c_str());

    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("          Dead Reckoning Navigation System (DRNS) Startup");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("");
    settings_.printCrwPayloadSettings();

    settings_.initializeIMU(&mpu_);

    // Write test data.
    // testFile.print(F(
    //     "abc,123,456,7.89\r\n"
    //     "def,-321,654,-9.87\r\n"
    //     "ghi,333,0xff,5.55"));
    // testFile.flush();

    Logger::notice("Example data written and file closed.");
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
    readTime_ = millis();

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Get values from the mpu IMU
    mpuRawAccel_       = mpu_.readRawAccel();
    mpuNormAccel_      = mpu_.readNormalizeAccel();
    mpuRawGyro_        = mpu_.readRawGyro();
    mpuNormGyro_       = mpu_.readNormalizeGyro();
    mpu6050Activities_ = mpu_.readActivites();
    mpuTemp_           = mpu_.readTemperature();

    roll_  = roll_ + mpuNormGyro_.XAxis * settings_.timeInterval_;
    pitch_ = pitch_ + mpuNormGyro_.YAxis * settings_.timeInterval_;
    yaw_   = yaw_ + mpuNormGyro_.ZAxis * settings_.timeInterval_;

    // Write MPU 6050 Data to the csv file
    trajectoryFile_.print(readTime_);
    trajectoryFile_.print(F("\n"));
    trajectoryFile_.print(mpuRawAccel_.XAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuRawAccel_.YAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuRawAccel_.ZAxis);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(mpuNormAccel_.XAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuNormAccel_.YAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuNormAccel_.ZAxis);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(mpuRawGyro_.XAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuRawGyro_.YAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuRawGyro_.ZAxis);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(mpuNormGyro_.XAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuNormGyro_.YAxis);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpuNormGyro_.ZAxis);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(roll_);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(pitch_);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(yaw_);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(mpu6050Activities_.isActivity);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isInactivity);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isPosActivityOnX);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isNegActivityOnX);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isPosActivityOnY);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isNegActivityOnY);
    trajectoryFile_.print(F(","));

    trajectoryFile_.print(mpu6050Activities_.isPosActivityOnZ);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isNegActivityOnZ);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isDataReady);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isOverflow);
    trajectoryFile_.print(F(","));
    trajectoryFile_.print(mpu6050Activities_.isFreeFall);
    trajectoryFile_.print(F(","));
    trajectoryFile_.println(mpuTemp_);

    delay((settings_.timeInterval_ * 1000) - (millis() - readTime_));
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

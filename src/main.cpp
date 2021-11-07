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
// Program Configuration (Pins, Sensors, Constants)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define PIN_D7 12

MPU6050 mpu_;
Settings settings_;

SdFs sd;
FsFile trajectoryFile_;
FsFile loggingFile_;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Setup Functions
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
    // if (loggingFile_.isOpen())
    // {
    //     loggingFile_.print(Logger::asString(level));
    //     loggingFile_.print(module);
    //     loggingFile_.println(message);
    //     loggingFile_.flush();
    // }

    Serial.print(Logger::asString(level));
    Serial.print(module);
    Serial.println(message);
}
void func()
{
    Serial.println("test");
}

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

    openFile(trajectoryFile_, settings_.getTrajectoryFilename());
    openFile(loggingFile_, settings_.getLoggingFilename());

    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("          Dead Reckoning Navigation System (DRNS) Startup");
    Logger::notice("---------------------------------------------------------------------");

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
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
}

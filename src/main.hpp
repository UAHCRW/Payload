/*
Retrieves sensor data and writes to a csv file on the sd card
Sensors:
  Magnetometer x1
  Accelerometer x3
  Gyroscope x3

Author: Charger Rocket Works 2021/2022 Team
Date: Fall 2021
*/
#include "ADXL357_Accelerometer.hpp"
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

Vector mpuNormAccel_;
Vector mpuNormGyro_;
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
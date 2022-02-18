/*
Retrieves sensor data and writes to a csv file on the sd card
Sensors:
  Accelerometer x3
  Gyroscope x3

Author: Charger Rocket Works 2021/2022 Team
Date: Fall 2021
*/
#include "ADXL357_Accelerometer.hpp"
#include "Arduino.h"
#include "IAM_20380.hpp"
#include "Logger.hpp"
#include "SPI.h"
#include "SdFat.h"
#include "settings.hpp"
#include <avr/interrupt.h>
#include <avr/io.h>
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
Settings settings_;
uint32_t readTime_{0};
SdFs sd;
FsFile trajectoryFile_;
FsFile loggingFile_;
volatile bool isrTriggered{false};
volatile bool isrMissed{false};
double lastFakeInterruptFire_{0.0};

// Sensors
ADXL357::Accelerometer acceleromter_;
ADXL357::AccelerometerConfig accelConfig_;
ADXL357::AccelerometerData accelData_;
IAM20380::Gyroscope gyro_;
IAM20380::GyroscopeConfig gyroConfig_;
IAM20380::GyroData gyroData_;

#define ACCELEROMETER_CHIP_SELECT 0
#define GYRO_CHIP_SELECT          10
#define BEAGLE_BONE_PULSE_PIN     38
#define CRW_SPI_CLOCK_SPEED       2e6

// Rename the UART object to make it more readable in the code since it is so close to Serial
#define BB_UART Serial8

// Functions
// ---------------------------------------------------------------------------------------------------------------------

/// \brief Interrupt service routine that is called on every beagle bone pulse
void isrRoutine();

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
void writeDataToCsv(FsFile& file, int16_t& data, bool endLine = false);
void writeDataToCsv(FsFile& file, float& data, bool endLine = false);
void writeDataToCsv(FsFile& file, double& data, bool endLine = false);
void writeDataToCsv(FsFile& file, ADXL357::AccelerometerData& data, bool endLine = false);
void writeDataToCsv(FsFile& file, IAM20380::GyroData& data, bool endLine = false);
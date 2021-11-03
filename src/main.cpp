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

//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))

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

#define testFileName "testFile.csv"

// Sd card class
SdFs sd;
FsFile testFile;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

    Serial.println("Mounting SD Card");

    // Initialize the SD.
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
        return;
    }

    Serial.println("SD Card Succesfully mounted");

    Serial.print("Attempting to open test file ");
    Serial.println(testFileName);

    // Create the file.
    if (!testFile.open(testFileName, FILE_WRITE))
    {
        error("open failed");
    }

    Serial.println("Test file open, writing some data.......");

    // Write test data.
    testFile.print(F(
        "abc,123,456,7.89\r\n"
        "def,-321,654,-9.87\r\n"
        "ghi,333,0xff,5.55"));

    testFile.close();

    Serial.println("Example data written and file closed.");
}

void loop()
{
    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    delay(500);
}

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
// SD Card Initialization
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

//------------------------------------------------------------------------------
// Program Configuration
//------------------------------------------------------------------------------
#define TEST_FILE_NAME    "testFile.csv"
#define LOGGING_FILE_NAME "logging.txt"
#define PIN_D7            12

// Sd card class
SdFs sd;
FsFile testFile;
FsFile loggingFile;

//------------------------------------------------------------------------------
// Setup Functions
//------------------------------------------------------------------------------
enum LogLevel
{
    UNKNOWN,
    ERROR,
    WARN,
    INFO,
    DEBUG1,
    DEBUG2
};

void logger(String msg, LogLevel level)
{
    String hdr{""};
    if (level == LogLevel::ERROR)
        hdr = "Error: ";
    else if (level == LogLevel::WARN)
        hdr = "Warn: ";
    else if (level == LogLevel::INFO)
        hdr = "Info: ";
    else if (level == LogLevel::DEBUG1)
        hdr = "Debug1: ";
    else if (level == LogLevel::DEBUG2)
        hdr = "Debug2: ";
    else
        hdr = "Unknown: ";

    msg = hdr + msg;

    if (loggingFile.isOpen())
    {
        loggingFile.print(String(msg));
        // Serial.println("logged message [" + String(msg) + "]");
    }
    // else
    //     Serial.println("Logging file not open, cannot log");

    Serial.println(msg);
}

void logger(String msg)
{
    logger(msg, LogLevel::UNKNOWN);
}

void openFile(FsFile& file, const char* filename)
{
    if (!file.open(filename, O_CREAT | O_WRITE))
        logger("Failed to open file [" + String(filename) + "]", LogLevel::ERROR);
    else
        logger("File [" + String(filename) + "] succesfully opened", LogLevel::INFO);
}

//------------------------------------------------------------------------------
// Setup Functions
//------------------------------------------------------------------------------
void setup()
{
    Serial.println("\n\n\n");

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_D7, INPUT_PULLUP); // Pushbutton
    Serial.begin(9600);

    logger("Mounting SD Card", LogLevel::INFO);

    // Initialize the SD.
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
        return;
    }

    logger("SD Card Succesfully mounted", LogLevel::INFO);

    openFile(testFile, TEST_FILE_NAME);
    openFile(loggingFile, LOGGING_FILE_NAME);

    // Write test data.
    testFile.print(F(
        "abc,123,456,7.89\r\n"
        "def,-321,654,-9.87\r\n"
        "ghi,333,0xff,5.55"));

    testFile.close();
    loggingFile.close();

    logger("Example data written and file closed.", LogLevel::INFO);
}

//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
void loop()
{
    // digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    // delay(500);

    if (digitalRead(PIN_D7))
    {
        Serial.println("Push button not pressed");
    }
    else
    {
        Serial.println("Push button pressed!!!!!!!!!!!!!!!!!!!");
    }
}

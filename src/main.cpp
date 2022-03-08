
#include "main.hpp"
#include "math.h"

// Comment / Uncomment these defines to configure how the teensy is used.
#define USE_SD_CARD        // Allows writing to the SD Card
#define LOG_TO_SERIAL      // Used for when using teensy without beagle bone to verify operation
#define LOG_TO_FILE        // Ignore writing to a file
#define INTERRUPT_NOT_USED // So we can use the teensy by itself.

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Setup()
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()
{
    Serial.begin(settings_.getBaudRate());
    BB_UART.begin(settings_.getBaudRate());
    SPI1.begin();

    Serial.println("\n\n\n\n");
    Logger::setOutputFunction(crwLogger);
    Logger::setLogLevel(settings_.getLoggingLevel());

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BEAGLE_BONE_PULSE_PIN, INPUT_PULLDOWN);
    pinMode(GYRO_CHIP_SELECT, OUTPUT);
    digitalWrite(GYRO_CHIP_SELECT, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    attachInterrupt(digitalPinToInterrupt(BEAGLE_BONE_PULSE_PIN), isrRoutine, CHANGE);

    delay(2000);

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
#else
    Logger::notice("CRW DRNS Software was built without SD Card. Files will not be written.")
#endif
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("          Dead Reckoning Navigation System (DRNS) Startup");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("");
    settings_.printCrwPayloadSettings();

    gyro_ = IAM20380::Gyroscope(GYRO_CHIP_SELECT, 14e6, gyroConfig_);

    bool initialized{false};

    initialized |= settings_.initiailzeCrwGyroscope(&gyro_);

    if (!initialized)
    {
        Logger::error("Initialization Failed");
        for (;;)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }

    Logger::notice("DRNS Initialization succesful");
    digitalWrite(LED_BUILTIN, HIGH);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
#ifdef INTERRUPT_NOT_USED
    if ((millis() - lastFakeInterruptFire_) > 100.0)
    {
        lastFakeInterruptFire_ = millis();
        isrRoutine();
    }
#endif
    if (isrMissed)
    {
        Logger::error("ISR trigged an interrupt before previous interrupt was finished being processed!!!!!!");
        isrMissed = false;
    }

    if (isrTriggered)
    {
        isrTriggered = false;
        gyro_.getStateFromLastFifoSample(gyroData_);

        // Write data to SD card if we have it turned on
#if defined(LOG_TO_FILE) && defined(USE_SD_CARD)
        writeDataToCsv(trajectoryFile_, gyroData_, true);
        trajectoryFile_.flush();
#endif

        // Write data to the main serial port on the teensy for optionally using teensy without beagle bone
#ifdef LOG_TO_SERIAL
        // Serial.print(gyroData_.x);
        // Serial.print(",  ");
        // Serial.print(gyroData_.y);
        // Serial.print(",  ");
        // Serial.println(gyroData_.z);
        float x{0.0}, y{0.0}, z{0.0};
        gyro_.getXYZ(x, y, z);
        Serial.print(x);
        Serial.print(",  ");
        Serial.print(y);
        Serial.print(",  ");
        Serial.println(z);
#endif

        // Send data to the beagle bone
        BB_UART.print(gyroData_.x);
        BB_UART.print(",");
        BB_UART.print(gyroData_.y);
        BB_UART.print(",");
        BB_UART.println(gyroData_.z);
    }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void isrRoutine()
{
    if (isrTriggered) isrMissed = true;
    isrTriggered = true;
}

//  --------------------------------------------------------------------------------------------------------------------
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

//  --------------------------------------------------------------------------------------------------------------------
void crwLogger(Logger::Level level, const char* module, const char* message)
{
#if defined(LOG_TO_FILE) && defined(USE_SD_CARD)
    if (loggingFile_.isOpen())
    {
        loggingFile_.print(Logger::asString(level));
        loggingFile_.print(module);
        loggingFile_.println(message);
        loggingFile_.flush();
    }
#endif

#ifdef LOG_TO_SERIAL
    Serial.print(Logger::asString(level));
    Serial.print(module);
    Serial.println(message);
#endif
}

//  --------------------------------------------------------------------------------------------------------------------
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
//  --------------------------------------------------------------------------------------------------------------------
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
//  --------------------------------------------------------------------------------------------------------------------
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

//  --------------------------------------------------------------------------------------------------------------------
void writeDataToCsv(FsFile& file, IAM20380::GyroData& data, bool endLine)
{
    file.print(data.x);
    file.print(",");
    file.print(data.y);
    file.print(",");
    file.print(data.z);
    file.print(",");
    file.print(data.xRaw);
    file.print(",");
    file.print(data.yRaw);
    if (endLine)
        file.println(data.zRaw);
    else
    {
        file.print(data.zRaw);
        file.print(",");
    }
}

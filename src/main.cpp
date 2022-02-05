
#include "main.hpp"
#include "math.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Setup()
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()
{
    Serial.begin(settings_.getBaudRate());

    Wire.begin();

    Logger::setOutputFunction(crwLogger);
    Logger::setLogLevel(settings_.getLoggingLevel());

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BEAGLE_BONE_PULSE_PIN, INPUT_PULLUP);
    pinMode(ACCELEROMETER_CHIP_SELECT, OUTPUT);
    pinMode(GYRO_CHIP_SELECT, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(BEAGLE_BONE_PULSE_PIN), isrRoutine, CHANGE);

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
    settings_.scanI2CNetwork();
    settings_.printCrwPayloadSettings();

    acceleromter_ = ADXL357::Accelerometer(ACCELEROMETER_CHIP_SELECT, CRW_SPI_CLOCK_SPEED, accelConfig_);
    gyro_         = IAM20380::Gyroscope(GYRO_CHIP_SELECT, CRW_SPI_CLOCK_SPEED, gyroConfig_);

    bool initialized{false};

    initialized &= settings_.initializeCrwAccelerometer(&acceleromter_);
    initialized &= settings_.initiailzeCrwGyroscope(&gyro_);

    if (!initialized)
    {
        for (;;)
        {
            Logger::error("Initialization Failed");
            delay(1);
        }
    }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
    if (isrTriggered)
    {
        acceleromter_.getStateFromFifoLastDataSample(accelData_);
        gyro_.getStateFromLastFifoSample(gyroData_);
#if defined(LOG_TO_FILE) && defined(USE_SD_CARD)
        writeDataToCsv(accelData_);
        writeDataToCsv(gyroData_, true);
        trajectoryFile_.flush();
#endif

#ifdef LOG_TO_SERIAL
        Serial.print(accelData_.x);
        Serial.print(",");
        Serial.print(accelData_.y);
        Serial.print(",");
        Serial.print(accelData_.z);
        Serial.print(",");
        Serial.print(gyroData_.x);
        Serial.print(",");
        Serail.print(gyroData_.y);
        Serial.print(",");
        Serial.println(gyroData_.z);
#endif
        isrTriggered = false;
    }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void isrRoutine()
{
    if (isrTriggered)
        Logger::error("ISR trigged an interrupt before previous interrupt was finished being processed!!!!!!");
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
void writeDataToCsv(FsFile& file, ADXL357::AccelerometerData& data, bool endLine)
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

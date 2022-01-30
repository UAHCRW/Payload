
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
    float mpuTemp{0};

    // Get values from the mpu IMU
    if (settings_.isMpu6050Initialized())
    {
        mpuNormAccel_      = mpu_.readScaledAccel();
        mpuNormGyro_       = mpu_.readNormalizeGyro();
        mpu6050Activities_ = mpu_.readActivites();
        mpuTemp            = mpu_.readTemperature();
    }

    // Get the values from the magnetometer
    if (settings_.isMagnetometerInitialized())
    {
        magnetomer_.read();
        magnetomer_.getEvent(&magEvent_);
    }

#ifdef USE_SD_CARD
    if (settings_.isMpu6050Initialized())
    {
        writeDataToCsv(trajectoryFile_, readTime_);
        writeDataToCsv(trajectoryFile_, mpuNormAccel_);
        writeDataToCsv(trajectoryFile_, mpuNormGyro_);
        writeDataToCsv(trajectoryFile_, mpu6050Activities_);
        writeDataToCsv(trajectoryFile_, mpuTemp, !settings_.isMagnetometerInitialized());
    }

    if (settings_.isMagnetometerInitialized()) writeDataToCsv(trajectoryFile_, magEvent_, true);

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
    Serial.print(magEvent_.magnetic.x);
    Serial.print(",");
    Serial.print(magEvent_.magnetic.y);
    Serial.print(",");
    Serial.println(magEvent_.magnetic.z);

    // We spent some time writing data and reading sensors so factor that in before next sample
    float timeUsed  = millis() - readTime_;
    float delayTime = settings_.getTimeIntervalMilliSeconds() - timeUsed;

    // Check to see if we missed a sample
    if (timeUsed > settings_.getTimeIntervalMilliSeconds())
    {
        int numSamplesMissed = (int)(timeUsed / 1000);
        String msg("Missed " + String(numSamplesMissed) + " samples. Time since read " + String(timeUsed) +
                   ". Time Interval " + String(numSamplesMissed));
        Logger::error(msg.c_str());
        delayTime = settings_.getTimeIntervalMilliSeconds() - fmod(timeUsed, settings_.getTimeIntervalMilliSeconds());
        Logger::notice(("Setting Delay for next sample to " + String(delayTime) + ".").c_str());
    }

    delay(delayTime);
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

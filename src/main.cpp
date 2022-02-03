
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
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
    readTime_ = millis();

#ifdef USE_SD_CARD

    trajectoryFile_.flush();
#endif

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

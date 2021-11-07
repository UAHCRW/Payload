#include "include/settings.hpp"
#include "stdio.h"
#include "string.h"

Settings::Settings()
    : trajectoryFileName_("trajectory.csv"), loggingFileName_("logging.txt"),
      loggingLevel_(Logger::Level::SILENT), baudRate_(115200),
      mpuAccelerometerRange_(mpu6050_range_t::MPU6050_RANGE_16G),
      mpuGyrometerRange_(mpu6050_dps_t::MPU6050_SCALE_2000DPS), mpu6050Initialized_(false)
{
}

void Settings::convertIntToChar(const char* message, char* buf, uint64_t val)
{
    strcpy(buf, message);
    char num[10];
    itoa(val, num, 10);
    strcat(buf, num);
    strcat(buf, "\0");
}

bool Settings::initializeIMU(MPU6050* sensor)
{
    char buf[100] = "";
    Logger::notice("Initializing MPU 6050 IMU");
    if (!sensor->begin(getMpuGyroRange(), getMpuAccelRange()))
    {
        Logger::error("Could not find a valid MPU6050 sensor, check wiring!");
        return false;
    }
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("                 MPU 6050 IMU Configuration");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice(" * Sleep Mode:            ");
    Logger::notice(sensor->getSleepEnabled() ? "   Enabled" : "   Disabled");
    Logger::notice("");

    Logger::notice(" * Motion Interrupt:     ");
    Logger::notice(sensor->getIntMotionEnabled() ? "   Enabled" : "   Disabled");
    Logger::notice("");

    Logger::notice(" * Zero Motion Interrupt:     ");
    Logger::notice(sensor->getIntZeroMotionEnabled() ? "   Enabled" : "   Disabled");
    Logger::notice("");

    Logger::notice(" * Free Fall Interrupt:       ");
    Logger::notice(sensor->getIntFreeFallEnabled() ? "   Enabled" : "   Disabled");
    Logger::notice("");

    Logger::notice(" * Motion Threshold:          ");
    convertIntToChar("   ", buf, sensor->getMotionDetectionThreshold());
    Logger::notice(buf);
    Logger::notice("");

    Logger::notice(" * Motion Duration:           ");
    convertIntToChar("   ", buf, sensor->getMotionDetectionDuration());
    Logger::notice(buf);
    Logger::notice("");

    Logger::notice(" * Zero Motion Threshold:     ");
    convertIntToChar("   ", buf, sensor->getZeroMotionDetectionThreshold());
    Logger::notice(buf);
    Logger::notice("");

    Logger::notice(" * Zero Motion Duration:      ");
    convertIntToChar("   ", buf, sensor->getZeroMotionDetectionDuration());
    Logger::notice(buf);
    Logger::notice("");

    Logger::notice("  Clock Source:          ");
    switch (sensor->getClockSource())
    {
        case MPU6050_CLOCK_KEEP_RESET:
            Logger::notice("   Stops the clock and keeps the timing generator in reset");
            break;
        case MPU6050_CLOCK_EXTERNAL_19MHZ:
            Logger::notice("   PLL with external 19.2MHz reference");
            break;
        case MPU6050_CLOCK_EXTERNAL_32KHZ:
            Logger::notice("   PLL with external 32.768kHz reference");
            break;
        case MPU6050_CLOCK_PLL_ZGYRO:
            Logger::notice("   PLL with Z axis gyroscope reference");
            break;
        case MPU6050_CLOCK_PLL_YGYRO:
            Logger::notice("   PLL with Y axis gyroscope reference");
            break;
        case MPU6050_CLOCK_PLL_XGYRO:
            Logger::notice("   PLL with X axis gyroscope reference");
            break;
        case MPU6050_CLOCK_INTERNAL_8MHZ:
            Logger::notice("   Internal 8MHz oscillator");
            break;
    }
    Logger::notice("");

    Logger::notice(" * Accelerometer:         ");
    switch (sensor->getRange())
    {
        case MPU6050_RANGE_16G:
            Logger::notice("   +/- 16 g");
            break;
        case MPU6050_RANGE_8G:
            Logger::notice("   +/- 8 g");
            break;
        case MPU6050_RANGE_4G:
            Logger::notice("   +/- 4 g");
            break;
        case MPU6050_RANGE_2G:
            Logger::notice("   +/- 2 g");
            break;
    }
    Logger::notice("");

    Logger::notice(" * Accelerometer offsets [XYZ]: ");
    convertIntToChar("   X: ", buf, sensor->getAccelOffsetX());
    Logger::notice(buf);
    convertIntToChar("   Y: ", buf, sensor->getAccelOffsetY());
    Logger::notice(buf);
    convertIntToChar("   Z: ", buf, sensor->getAccelOffsetZ());
    Logger::notice(buf);

    Logger::notice(" * Accelerometer power delay: ");
    switch (sensor->getAccelPowerOnDelay())
    {
        case MPU6050_DELAY_3MS:
            Logger::notice("   3ms");
            break;
        case MPU6050_DELAY_2MS:
            Logger::notice("   2ms");
            break;
        case MPU6050_DELAY_1MS:
            Logger::notice("   1ms");
            break;
        case MPU6050_NO_DELAY:
            Logger::notice("   0ms");
            break;
    }
    Logger::notice("");

    Logger::notice(" * Gyroscope:         ");
    switch (sensor->getScale())
    {
        case MPU6050_SCALE_2000DPS:
            Logger::notice("   2000 dps");
            break;
        case MPU6050_SCALE_1000DPS:
            Logger::notice("   1000 dps");
            break;
        case MPU6050_SCALE_500DPS:
            Logger::notice("   500 dps");
            break;
        case MPU6050_SCALE_250DPS:
            Logger::notice("   250 dps");
            break;
    }
    Logger::notice("");

    Logger::notice(" * Gyroscope offsets [XYZ]: ");
    convertIntToChar("   X: ", buf, sensor->getGyroOffsetX());
    Logger::notice(buf);
    convertIntToChar("   Y: ", buf, sensor->getGyroOffsetY());
    Logger::notice(buf);
    convertIntToChar("   Z: ", buf, sensor->getGyroOffsetZ());
    Logger::notice(buf);
    Logger::notice("");

    return true;
}
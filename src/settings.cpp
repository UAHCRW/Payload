#include "include/settings.hpp"
#include "stdio.h"
#include "string.h"

Settings::Settings()
    : trajectoryFileName_("trajectory.csv"), loggingFileName_("logging.txt"), loggingLevel_(Logger::Level::SILENT),
      baudRate_(115200), mpuAccelerometerRange_(mpu6050_range_t::MPU6050_RANGE_16G),
      mpuGyrometerRange_(mpu6050_dps_t::MPU6050_SCALE_2000DPS), mpu6050Initialized_(false)
{
}

bool Settings::initializeIMU(MPU6050* sensor)
{
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
    Logger::notice(sensor->getSleepEnabled() ? "   Enabled\n" : "   Disabled\n");

    Logger::notice(" * Motion Interrupt:     ");
    Logger::notice(sensor->getIntMotionEnabled() ? "   Enabled\n" : "   Disabled\n");

    Logger::notice(" * Zero Motion Interrupt:     ");
    Logger::notice(sensor->getIntZeroMotionEnabled() ? "   Enabled\n" : "   Disabled\n");

    Logger::notice(" * Free Fall Interrupt:       ");
    Logger::notice(sensor->getIntFreeFallEnabled() ? "   Enabled\n" : "   Disabled\n");

    Logger::notice(" * Motion Threshold:          ");
    Logger::notice(("\t" + String(sensor->getMotionDetectionThreshold()) + "\n").c_str());

    Logger::notice(" * Motion Duration:           ");
    Logger::notice(("\t" + String(sensor->getMotionDetectionDuration()) + "\n").c_str());

    Logger::notice(" * Zero Motion Threshold:     ");
    Logger::notice(("\t" + String(sensor->getZeroMotionDetectionThreshold()) + "\n").c_str());

    Logger::notice(" * Zero Motion Duration:      ");
    Logger::notice(("\t" + String(sensor->getZeroMotionDetectionDuration()) + "\n").c_str());

    Logger::notice("  Clock Source:          ");
    switch (sensor->getClockSource())
    {
        case MPU6050_CLOCK_KEEP_RESET: Logger::notice("\tStops the clock and keeps timing generator in reset\n"); break;
        case MPU6050_CLOCK_EXTERNAL_19MHZ: Logger::notice("\tPLL with external 19.2MHz reference\n"); break;
        case MPU6050_CLOCK_EXTERNAL_32KHZ: Logger::notice("\tPLL with external 32.768kHz reference\n"); break;
        case MPU6050_CLOCK_PLL_ZGYRO: Logger::notice("\tPLL with Z axis gyroscope reference\n"); break;
        case MPU6050_CLOCK_PLL_YGYRO: Logger::notice("\tPLL with Y axis gyroscope reference\n"); break;
        case MPU6050_CLOCK_PLL_XGYRO: Logger::notice("\tPLL with X axis gyroscope reference\n"); break;
        case MPU6050_CLOCK_INTERNAL_8MHZ: Logger::notice("\tInternal 8MHz oscillator\n"); break;
    }

    Logger::notice(" * Accelerometer:         ");
    switch (sensor->getRange())
    {
        case MPU6050_RANGE_16G: Logger::notice("\t+/- 16 g\n"); break;
        case MPU6050_RANGE_8G: Logger::notice("\t+/- 8 g\n"); break;
        case MPU6050_RANGE_4G: Logger::notice("\t+/- 4 g\n"); break;
        case MPU6050_RANGE_2G: Logger::notice("\t+/- 2 g\n"); break;
    }

    Logger::notice(" * Accelerometer offsets [XYZ]: ");
    Logger::notice(("\tX: " + String(sensor->getAccelOffsetX())).c_str());
    Logger::notice(("\tY: " + String(sensor->getAccelOffsetY())).c_str());
    Logger::notice(("\tZ: " + String(sensor->getAccelOffsetZ()) + "\n").c_str());

    Logger::notice(" * Accelerometer power delay: ");
    switch (sensor->getAccelPowerOnDelay())
    {
        case MPU6050_DELAY_3MS: Logger::notice("\t3ms\n"); break;
        case MPU6050_DELAY_2MS: Logger::notice("\t2ms\n"); break;
        case MPU6050_DELAY_1MS: Logger::notice("\t1ms\n"); break;
        case MPU6050_NO_DELAY: Logger::notice("\t0ms\n"); break;
    }

    Logger::notice(" * Gyroscope:         ");
    switch (sensor->getScale())
    {
        case MPU6050_SCALE_2000DPS: Logger::notice("\t2000 dps\n"); break;
        case MPU6050_SCALE_1000DPS: Logger::notice("\t1000 dps\n"); break;
        case MPU6050_SCALE_500DPS: Logger::notice("\t500 dps\n"); break;
        case MPU6050_SCALE_250DPS: Logger::notice("\t250 dps\n"); break;
    }

    Logger::notice(" * Gyroscope offsets [XYZ]: ");
    Logger::notice(("\t" + String(sensor->getGyroOffsetX())).c_str());
    Logger::notice(("\t" + String(sensor->getGyroOffsetY())).c_str());
    Logger::notice(("\t" + String(sensor->getGyroOffsetZ()) + "\n").c_str());

    return true;
}
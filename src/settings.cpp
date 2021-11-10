#include "settings.hpp"
#include "stdio.h"
#include "string.h"

Settings::Settings()
    : trajectoryFileName_("trajectory.csv"), loggingFileName_("logging.txt"), loggingLevel_(Logger::Level::VERBOSE),
      baudRate_(115200), samplingFreq_(100.0), timeInterval_(0.0),
      mpuAccelerometerRange_(mpu6050_range_t::MPU6050_RANGE_16G),
      mpuGyrometerRange_(mpu6050_dps_t::MPU6050_SCALE_2000DPS), mpu6050Initialized_(false),
      magnetometerInitialized_(false)
{
    timeInterval_ = 1 / samplingFreq_;
}

void Settings::scanI2CNetwork()
{
    Logger::notice("Scanning I2C network for peripherals");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            String out = "I2C device found at address 0x" + String(address, HEX) + " !";
            if (address < 16) out = "I2C device found at address 0x0" + String(address, HEX) + " !";
            Logger::notice(out.c_str());
            nDevices++;
        }
        else if (error == 4)
        {
            String out = "Unknown error at address 0x" + String(address, HEX) + " !";
            if (address < 16) out = "I2C device found at address 0x0" + String(address, HEX) + " !";
            Logger::notice(out.c_str());
        }
    }
    if (nDevices == 0)
        Logger::notice("No I2C devices found\n");
    else
        Logger::notice("done\n");
}

void Settings::printCrwPayloadSettings()
{
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("                 CRW 2021 - 2022 Payload Configuration");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice(("\tTrajectory Filename: " + trajectoryFileName_).c_str());
    Logger::notice(("\tLogging Filename:    " + loggingFileName_).c_str());
    Logger::notice(("\tLogging Level:       " + String(Logger::asString(loggingLevel_))).c_str());
    Logger::notice(("\tBaud Rate:           " + String(baudRate_)).c_str());
    Logger::notice(("\tSampling Freq [Hz]:  " + String(samplingFreq_)).c_str());
    Logger::notice(("\tTime Interval:       " + String(timeInterval_)).c_str());
    Logger::notice("");
}

bool Settings::initializeIMU(MPU6050* sensor)
{
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("                 MPU 6050 IMU Configuration");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("Initializing MPU 6050 IMU");
    if (!sensor->begin(mpuGyrometerRange_, mpuAccelerometerRange_))
    {
        Logger::error("Failed to initialize MPU6050 IMU. Sensor not found on I2C network");
        return false;
    }
    Logger::notice("MPU6050 Succesfully Initialized");
    Logger::notice("");
    mpu6050Initialized_ = true;

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

    Logger::notice("  Clock Source:");
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

    Logger::notice(" * Accelerometer [g]:");
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

    Logger::notice(" * Gyroscope [dps]:");
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

    return mpu6050Initialized_;
}

bool Settings::initializeMagnetometer(Adafruit_LIS3MDL* sensor, uint8_t pin)
{
    if (!sensor->begin_SPI(pin))
    {
        Logger::error("Failed to initialize LIS3MDL Magnetometer. Sensor not found on I2C network");
        return magnetometerInitialized_;
    }
    magnetometerInitialized_ = true;

    Logger::notice("---------------------------------------------------------------------");
    Logger::notice("                 MPU 6050 IMU Configuration");
    Logger::notice("---------------------------------------------------------------------");
    Logger::notice(" * Operation Mode:");
    switch (sensor->getOperationMode())
    {
        case LIS3MDL_CONTINUOUSMODE: Logger::notice("\tContinuous\n"); break;
        case LIS3MDL_SINGLEMODE: Logger::notice("\tSingle mode\n"); break;
        case LIS3MDL_POWERDOWNMODE: Logger::notice("\tPower-down\n"); break;
    }

    Logger::notice(" * Data Rate [Hz]:");
    switch (sensor->getDataRate())
    {
        case LIS3MDL_DATARATE_0_625_HZ: Logger::notice("\t0.625 Hz\n"); break;
        case LIS3MDL_DATARATE_1_25_HZ: Logger::notice("\t1.25 Hz\n"); break;
        case LIS3MDL_DATARATE_2_5_HZ: Logger::notice("\t2.5 Hz\n"); break;
        case LIS3MDL_DATARATE_5_HZ: Logger::notice("\t5 Hz\n"); break;
        case LIS3MDL_DATARATE_10_HZ: Logger::notice("\t10 Hz\n"); break;
        case LIS3MDL_DATARATE_20_HZ: Logger::notice("\t20 Hz\n"); break;
        case LIS3MDL_DATARATE_40_HZ: Logger::notice("\t40 Hz\n"); break;
        case LIS3MDL_DATARATE_80_HZ: Logger::notice("\t80 Hz\n"); break;
        case LIS3MDL_DATARATE_155_HZ: Logger::notice("\t155 Hz\n"); break;
        case LIS3MDL_DATARATE_300_HZ: Logger::notice("\t300 Hz\n"); break;
        case LIS3MDL_DATARATE_560_HZ: Logger::notice("\t560 Hz\n"); break;
        case LIS3MDL_DATARATE_1000_HZ: Logger::notice("\t1000 Hz\n"); break;
    }

    Logger::notice(" * Range [gauss]:");
    switch (sensor->getRange())
    {
        case LIS3MDL_RANGE_4_GAUSS: Logger::notice("\t+-4 gauss\n"); break;
        case LIS3MDL_RANGE_8_GAUSS: Logger::notice("\t+-8 gauss\n"); break;
        case LIS3MDL_RANGE_12_GAUSS: Logger::notice("\t+-12 gauss\n"); break;
        case LIS3MDL_RANGE_16_GAUSS: Logger::notice("\t+-16 gauss\n"); break;
    }

    return magnetometerInitialized_;
}
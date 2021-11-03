#ifndef ADXL356_ACCELEROMETER_HPP
#define ADXL356_ACCELEROMETER_HPP

#include <stdint.h>

enum ADXL356Reg : uint8_t
{
    DEVID_AD     = 0x00,
    DEVID_MST    = 0X01,
    PARTID       = 0X02,
    REVID        = 0X03,
    STATUS       = 0X04,
    FIFO_ENTRIES = 0X05,
    TEMP2        = 0X06,
    TEMP1        = 0X07,
    XDATA3       = 0X08,
    XDATA2       = 0X09,
    XDATA1       = 0X0A,
    YDATA3       = 0X0B,
    YDATA2       = 0X0C,
    YDATA1       = 0X0D,
    ZDATA3       = 0X0E,
    ZDATA2       = 0X0F,
    ZDATA1       = 0X10,
    FIFO_DATA    = 0X11,
    OFFSET_X_H   = 0X1E,
    OFFSET_X_L   = 0X1F,
    OFFSET_Y_H   = 0X20,
    OFFSET_Y_L   = 0X21,
    OFFSET_Z_H   = 0X22,
    OFFSET_Z_L   = 0X23,
    ACT_EN       = 0X24,
    ACT_THRESH_H = 0X25,
    ACT_THRESH_L = 0X26,
    ACT_COUNT    = 0X27,
    FILTER       = 0X28,
    FIFO_SAMPLES = 0X29,
    INT_MAP      = 0X2A,
    SYNC         = 0X2B,
    RANGE        = 0X2C,
    POWER_CTL    = 0X2D,
    SELF_TEST    = 0X2E,
    RESET        = 0X2F,
};

enum ADXL356_HP_Filter : uint8_t
{
    // These values are multiplied by the output data rate (ODR)
    // Format for 1.5545x10^-4 X ODR -> HP_1_5545 where _ represents a decimal
    NO_FILTER = 0x00,
    HP_24_7   = 0x01,
    HP_6_2084 = 0x02,
    HP_1_5545 = 0x03,
    HP_3862   = 0x04,
    HP_0954   = 0x05,
    HP_0238   = 0x6,
};

enum ADXL356__ODR_LP_FILTER : uint8_t
{
    // Output Data Rate (ODR) -> Low Pass Filter (LP) "_" in a number represents a decimal
    // All values in Hz
    ODR_4000_LP_1000    = 0x00,
    ODR_2000_LP_500     = 0x01,
    ODR_1000_LP_250     = 0x02,
    ODR_500_LP_125      = 0x03,
    ODR_250_LP_62_5     = 0x04,
    ODR_125_LP_31_25    = 0x05,
    ODR_62_5_LP_15_625  = 0x06,
    ODR_31_25_LP_7_813  = 0x07,
    ODR_15_625_LP_3_906 = 0x08,
    ODR_7_813_LP_1_953  = 0x09,
    ODR_3_906_LP_0_977  = 0x0A
};

class ADXL356 final
{
    public:
    ADXL356(){};
    ~ADXL356(){};

    double getAccelX();
    double getAccelY();
    double getAccelZ();
    double getTemperature();

    private:
    uint8_t resetCode_ = 0x52;
};

#endif
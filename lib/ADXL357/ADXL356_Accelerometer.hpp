#ifndef ADXL356_ACCELEROMETER_HPP
#define ADXL356_ACCELEROMETER_HPP

#include <stdint.h>

// General Register Values
#define ADXL356_REG_DEVID_AD     0x00
#define ADXL356_REG_DEVID_MST    0X01
#define ADXL356_REG_PARTID       0X02
#define ADXL356_REG_REVID        0X03
#define ADXL356_REG_STATUS       0X04
#define ADXL356_REG_FIFO_ENTRIES 0X05
#define ADXL356_REG_TEMP2        0X06
#define ADXL356_REG_TEMP1        0X07
#define ADXL356_REG_XDATA3       0X08
#define ADXL356_REG_XDATA2       0X09
#define ADXL356_REG_XDATA1       0X0A
#define ADXL356_REG_YDATA3       0X0B
#define ADXL356_REG_YDATA2       0X0C
#define ADXL356_REG_YDATA1       0X0D
#define ADXL356_REG_ZDATA3       0X0E
#define ADXL356_REG_ZDATA2       0X0F
#define ADXL356_REG_ZDATA1       0X10
#define ADXL356_REG_FIFO_DATA    0X11
#define ADXL356_REG_OFFSET_X_H   0X1E
#define ADXL356_REG_OFFSET_X_L   0X1F
#define ADXL356_REG_OFFSET_Y_H   0X20
#define ADXL356_REG_OFFSET_Y_L   0X21
#define ADXL356_REG_OFFSET_Z_H   0X22
#define ADXL356_REG_OFFSET_Z_L   0X23
#define ADXL356_REG_ACT_EN       0X24
#define ADXL356_REG_ACT_THRESH_H 0X25
#define ADXL356_REG_ACT_THRESH_L 0X26
#define ADXL356_REG_ACT_COUNT    0X27
#define ADXL356_REG_FILTER       0X28
#define ADXL356_REG_FIFO_SAMPLES 0X29
#define ADXL356_REG_INT_MAP      0X2A
#define ADXL356_REG_SYNC         0X2B
#define ADXL356_REG_RANGE        0X2C
#define ADXL356_REG_POWER_CTL    0X2D
#define ADXL356_REG_SELF_TEST    0X2E
#define ADXL356_REG_RESET        0X2F

// Register Values for High pass filter
// These values are multiplied by the output data rate (ODR)
// Format for 1.5545x10^-4 X ODR -> HP_1_5545 where _ represents a decimal
#define ADXL356_HP_FILTER_NO_FILTER 0x00
#define ADXL356_HP_FILTER_HP_24_7   0x01
#define ADXL356_HP_FILTER_HP_6_2084 0x02
#define ADXL356_HP_FILTER_HP_1_5545 0x03
#define ADXL356_HP_FILTER_HP_3862   0x04
#define ADXL356_HP_FILTER_HP_0954   0x05
#define ADXL356_HP_FILTER_HP_0238   0x6

// Output Data Rate (ODR) -> Low Pass Filter (LP) "_" in a number represents a decimal
// All values in Hz
#define ADXL356_ODR_4000_LP_1000    0x00
#define ADXL356_ODR_2000_LP_500     0x01
#define ADXL356_ODR_1000_LP_250     0x02
#define ADXL356_ODR_500_LP_125      0x03
#define ADXL356_ODR_250_LP_62_5     0x04
#define ADXL356_ODR_125_LP_31_25    0x05
#define ADXL356_ODR_62_5_LP_15_625  0x06
#define ADXL356_ODR_31_25_LP_7_813  0x07
#define ADXL356_ODR_15_625_LP_3_906 0x08
#define ADXL356_ODR_7_813_LP_1_953  0x09
#define ADXL356_ODR_3_906_LP_0_977  0x0A

// Misc
#define ADXL356_RESET_CODE 0X52

struct AccelerometerData
{
    double x;
    double y;
    double z;
    double temp;
    // dou
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
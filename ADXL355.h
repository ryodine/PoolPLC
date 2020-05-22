#ifndef ADXL355__H_GUARD
#define ADXL355__H_GUARD

#include <SPI.h>

// READ registers
#define ADXL355__REG_XDATA3                   0x08
#define ADXL355__REG_ACCELEROMETER_DATA_BEGIN ADXL355__REG_XDATA3
#define ADXL255__REG_STATUS                   0x04

// WRITE Registers
#define ADXL355__REG_POWER_CONTROL 0x2D
#define ADXL355__REG_RESET         0x2F
#define ADXL355__REG_FILTER        0x28
#define ADXL355__REG_RANGE         0x2C

// Internal Constants
#define AXL355__CONST_MEASURE_MODE 0x06
#define AXL355__SPI_WRITEBIT       0x00
#define AXL355__SPI_READBIT        0x01

// Public Constants
#define ADXL355_RANGE_2G 0x01
#define ADXL355_RANGE_4G 0x02
#define ADXL355_RANGE_8G 0x03

#define ADXL355_STATUS_DATA_READY 0b00000001
#define ADXL355_STATUS_FIFO_FULL  0b00000010
#define ADXL355_STATUS_FIFO_OVR   0b00000100
#define ADXL355_STATUS_ACTIVITY   0b00001000
#define ADXL355_STATUS_NVM_BUSY   0b00010000

#define ADXL355_FILTER_LPF_4HZ_ODR  0b00001010
#define ADXL355_FILTER_LPF_8HZ_ODR  0b00001001
#define ADXL355_FILTER_LPF_16HZ_ODR 0b00001000
#define ADXL355_FILTER_OFF          0x000000

typedef struct {
    double x;
    double y;
    double z;
} ADXL355Measurement;

class ADXL355 {
  private:
    int chipselect;
    int chipselect2;
    SPISettings settings;
    long axes[3];
    int range;

    void spi_multibyte_read(byte *buffer, int numBytes, byte startaddress);
    void spi_writebyte(byte address, byte toWrite);
    byte spi_readbyte(byte address);
    void setCS(bool active);

  public:
    ADXL355(int cs, int speed = 5000000, int cs2 = -1)
        : chipselect(cs), chipselect2(cs2),
          settings(SPISettings(speed, MSBFIRST, SPI_MODE0)){};
    boolean begin(byte range, byte filter = ADXL355_FILTER_OFF);
    void takeSample();
    byte getStatus();
    bool dataReady() { return getStatus() & ADXL355_STATUS_DATA_READY; };
    ADXL355Measurement getSample();
};

#endif

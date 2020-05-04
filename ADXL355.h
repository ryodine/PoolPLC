#ifndef ADXL355__H_GUARD
#define ADXL355__H_GUARD

#include <SPI.h>

// READ registers
#define ADXL355__REG_XDATA3 0x08
#define ADXL355__REG_ACCELEROMETER_DATA_BEGIN ADXL355__REG_XDATA3

// WRITE Registers
#define ADXL355__REG_POWER_CONTROL 0x2D
#define ADXL355__REG_RESET 0x2F
#define ADXL355__REG_FILTER 0x28
#define ADXL355__REG_RANGE 0x2C

// Internal Constants
#define AXL355__CONST_MEASURE_MODE 0x06
#define AXL355__SPI_WRITEBIT 0x00
#define AXL355__SPI_READBIT 0x01

// Public Constants
#define ADXL355_RANGE_2G 0x01
#define ADXL355_RANGE_4G 0x02
#define ADXL355_RANGE_8G 0x03

#define ADXL355_FILTER_LPF_LOW_FREQ 0b00001010
#define ADXL355_FILTER_OFF 0x000000

typedef struct{
  double x;
  double y;
  double z;
} ADXL355Measurement;

class ADXL355 {
  private:
    int chipselect;
    SPISettings settings;
    long axes[3];
    int range;

    void spi_multibyte_read(byte* buffer, int numBytes, byte startaddress);
    void spi_writebyte(byte address, byte toWrite);
    byte spi_readbyte(byte address);
    
  public:
    ADXL355(int cs, int speed=9000000) 
      : chipselect(cs), settings(SPISettings(speed, MSBFIRST, SPI_MODE0)) {};
    boolean begin(byte range, byte filter=ADXL355_FILTER_OFF);
    void takeSample();
    ADXL355Measurement getSample();
};

#endif

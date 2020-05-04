#include "ADXL355.h"

void ADXL355::spi_writebyte(byte address, byte toWrite)
{
  SPI.beginTransaction(settings);
  digitalWrite(chipselect, LOW);
  
  SPI.transfer((address<<1) | AXL355__SPI_WRITEBIT);
  SPI.transfer(toWrite);
  
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();
}

byte ADXL355::spi_readbyte(byte address)
{
  byte result = 0x0;
  SPI.beginTransaction(settings);
  digitalWrite(chipselect, LOW);
  
  SPI.transfer((address<<1) | AXL355__SPI_READBIT);
  result = SPI.transfer(0x00);
  
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();
  return result;
}

void ADXL355::spi_multibyte_read(byte* buf, int buffersize, byte startaddress)
{
  SPI.beginTransaction(settings);
  digitalWrite(chipselect, LOW);
  
  SPI.transfer((byte) ((startaddress<<1) | AXL355__SPI_READBIT));
  for(int i = 0; i < buffersize; i++) {
    buf[i] = SPI.transfer(0x0);
  }
  
  digitalWrite(chipselect, HIGH);
  SPI.endTransaction();
}

boolean ADXL355::begin(byte range, byte filter=ADXL355_FILTER_OFF)
{
  this->range = range;
  pinMode(chipselect, OUTPUT);
  digitalWrite(chipselect, LOW);
  
  spi_writebyte(ADXL355__REG_RESET, 0x00);
  spi_writebyte(ADXL355__REG_RANGE, range);
  spi_writebyte(ADXL355__REG_FILTER, filter);
  spi_writebyte(ADXL355__REG_POWER_CONTROL, AXL355__CONST_MEASURE_MODE);
  byte adbyte = spi_readbyte(0x00);
  
  digitalWrite(chipselect, HIGH);
  return adbyte == 0xAD;
}

void ADXL355::takeSample()
{
  byte bytebuf[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  spi_multibyte_read(bytebuf, 9, ADXL355__REG_ACCELEROMETER_DATA_BEGIN);
  
  unsigned long buf[9];
  for (int i = 0; i < 9; i++)
    buf[i] = (int)bytebuf[i];
  
  axes[0] = (buf[2] | buf[1] << 8 | buf[0] << 16) & 0xFFFFFF;
  axes[1] = (buf[5] | buf[4] << 8 | buf[3] << 16) & 0xFFFFFF;
  axes[2] = (buf[8] | buf[7] << 8 | buf[6] << 16) & 0xFFFFFF;
  long mask = 1;
  mask = mask << 23;

  for (int i = 0 ; i < 3; i++) {
    if (axes[i] > mask) {
      axes[i] = -((mask<<1)-axes[i]);
    }
  }
}

ADXL355Measurement ADXL355::getSample()
{
  long scale = ((long)2 << ((3 - range) + 5)) * 1000;
  ADXL355Measurement measure;
  measure.x = axes[0] / (1.0*scale);
  measure.y = axes[1] / (1.0*scale);
  measure.z = axes[2] / (1.0*scale);

  return measure;
}

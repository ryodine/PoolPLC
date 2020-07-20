/**
 * @file ADXL355.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Interface for Analog Devices ADXL355 3-Axis high-accuracy MEMS
 * accelerometer
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

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

/**
 * @brief Represents a 3-axis measurement from the accelerometer
 */
typedef struct {
    double x;
    double y;
    double z;
} ADXL355Measurement;

/**
 * @brief Interface for communicating with the ADXL355 chip over SPI
 */
class ADXL355 {
  public:
    /**
     * @brief Construct a new ADXL355 object
     *
     * @param cs SPI chip select pin
     * @param speed SPI speed in Hz (to be used in SPISettings), default is
     * 5000000Hz.
     * @param cs2 If there is a second chip select, for example when used over a
     * shared bus like isoSPI on the LTC6820 Chip, this can be used, otherwise
     * it can be set to a negative number (default) to disable it
     */
    ADXL355(int cs, int speed = 5000000, int cs2 = -1)
        : chipselect(cs), chipselect2(cs2),
          settings(SPISettings(speed, MSBFIRST, SPI_MODE0)){};

    /**
     * @brief Start the ADXL355 accelerometer with a filter and a operating
     * range.
     *
     * The operating range should be one of ADXL355_RANGE_2G, ADXL355_RANGE_4G,
     * or ADXL355_RANGE_8G. These are most to least sensitive, respectively.
     *
     * The filter should be one of the constants prefixed ADXL355_FILTER_. Use
     * ADXL355_FILTER_LPF_4HZ_ODR to filter out almost all noise except for
     * inertial acceleration.
     *
     * @param range the range for the accelerometer to operate at.
     * @param filter the digital filter, if any, to use in the accelerometer.
     * @return true if initialization is successful
     * @return false if initialization is unsuccessful
     */
    bool begin(byte range, byte filter = ADXL355_FILTER_OFF);

    /**
     * @brief Take a sample over SPI and cache it
     */
    void takeSample();

    /**
     * @brief Query the status of the accelerometer
     *
     * Use the constants prefixed ADXL355_STATUS_ to mask out specific status
     * bits.
     *
     * @return byte status byte.
     */
    byte getStatus();

    /**
     * @brief Check if the accelerometer has data ready.
     *
     * @return true if the accelerometer has a reading
     * @return false if the accelerometer does not have a reading yet
     */
    bool dataReady() { return getStatus() & ADXL355_STATUS_DATA_READY; };

    /**
     * @brief return the cached measurement - use after calling takeSample()
     *
     * @return ADXL355Measurement the cahced measurement
     */
    ADXL355Measurement getSample();

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
};

#endif

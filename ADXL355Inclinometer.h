/**
 * @file ADXL355Inclinometer.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Treats an ADXL355 accelerometer like an inclinometer
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef ADLX355_INCLINOMETER_MODULE_H
#define ADLX355_INCLINOMETER_MODULE_H

#include "ADXL355.h"
#include "AccelerometerFiltering.h"
#include "InclinometerInterface.h"

namespace Inclinometer {

/**
 * @brief An implementation of an InclinometerDataSource using an ADXL355
 * accelerometer and some math.
 */
class ADXL355Inclinometer : public InclinometerDataSource {
  public:
    /**
     * @brief Construct a new ADXL355Inclinometer object
     *
     * @param cs chip select 1
     * @param cs2 chip select 2 (if used)
     * @param ewmaAlpha exponentially weighted moving average filtering
     * coefficient
     * @param filter the digital filter within the ADXL355 to use
     * @param speed the SPI speed to use (Hz)
     */
    ADXL355Inclinometer(int cs, int cs2 = -1, double ewmaAlpha = 0.1,
                        byte filter = ADXL355_FILTER_LPF_4HZ_ODR,
                        int speed = 5000000 /*5000000 625000*/)
        : accel(cs, speed, cs2), filter(filter),
          movingAverageFilter(0.0, ewmaAlpha){};

    //! Inclinometer Data Source Interface Methods

    bool begin() override { return accel.begin(ADXL355_RANGE_2G, filter); }
    bool hasData() override { return accel.dataReady(); };
    Eigen::Vector2d getData() override;

  private:
    ADXL355 accel;
    AccelerometerFilterMovingAverage movingAverageFilter;
    byte filter;
};
}; // namespace Inclinometer

#endif
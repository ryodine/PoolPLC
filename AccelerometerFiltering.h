/**
 * @file AccelerometerFiltering.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Digital filters for 3-axis accelerometer data
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef ACCELEROMETER_FILTERING_GUARD_H
#define ACCELEROMETER_FILTERING_GUARD_H
#include "ADXL355.h"
#include "MovingAverage.h"

/**
 * @brief Exponentially-weighted moving average for 3-axis accelerometers
 *
 * @see MovingAverage
 */
class AccelerometerFilterMovingAverage {
  public:
    AccelerometerFilterMovingAverage(float startingPoint, float alpha)
        : avgs({MovingAverage(startingPoint, alpha),
                MovingAverage(startingPoint, alpha),
                MovingAverage(startingPoint, alpha)}){};
    void addData(ADXL355Measurement measure)
    {
        for (int i = 0; i < 3; i++) {
            avgs[i].addPoint(((double *)&measure)[i]);
        }
    };
    ADXL355Measurement getAverage()
    {
        ADXL355Measurement avg;
        for (int i = 0; i < 3; i++) {
            ((double *)&avg)[i] = avgs[i].getAverage();
        }
        return avg;
    };

  private:
    MovingAverage avgs[3];
};

#endif

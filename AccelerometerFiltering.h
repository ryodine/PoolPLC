#ifndef ACCELEROMETER_FILTERING_GUARD_H
#define ACCELEROMETER_FILTERING_GUARD_H
#include "ADXL355.h"
#include "MovingAverage.h"


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

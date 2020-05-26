#ifndef ADLX355_INCLINOMETER_MODULE_H
#define ADLX355_INCLINOMETER_MODULE_H

#include "ADXL355.h"
#include "AccelerometerFiltering.h"
#include "InclinometerInterface.h"

namespace Inclinometer {
class ADXL355Inclinometer : public InclinometerDataSource {
  public:
    ADXL355Inclinometer(int cs, int cs2, double ewmaAlpha = 0.1,
                        byte filter = ADXL355_FILTER_LPF_4HZ_ODR,
                        int speed = 5000000/*5000000 625000*/)
        : accel(cs, speed, cs2), filter(filter),
          movingAverageFilter(0.0, ewmaAlpha){};

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
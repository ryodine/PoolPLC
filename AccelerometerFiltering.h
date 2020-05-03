#ifndef ACCELEROMETER_FILTERING_GUARD_H
#define ACCELEROMETER_FILTERING_GUARD_H
#include "MovingAverage.h"

class AccelerometerFilterMovingAverage {
  public:
    AccelerometerFilterMovingAverage(float startingPoint, float alpha)
      : avgs({ MovingAverage(startingPoint, alpha),MovingAverage(startingPoint, alpha),MovingAverage(startingPoint, alpha) }){};
    void addData(ADXL355Measurement measure) {
      for (int i = 0; i < 3; i++) {
        avgs[i].addPoint(((double*)&measure)[i]);
      }
    };
    ADXL355Measurement getAverage() {
      ADXL355Measurement avg;
      for (int i = 0; i < 3; i++) {
        ((double*)&avg)[i] = avgs[i].getAverage();
      }
      return avg;
    };
  private:
    MovingAverage avgs[3];
};

class Vect3D {
  public:
    double components[3];
    Vect3D(double x, double y, double z) : components({x, y, z}) {};
};

class UnitVectorizeMeasurement {
  public:
    static Vect3D vectorize(ADXL355Measurement measure) {
      double scale = sqrt(pow(measure.x/16.0, 2) + pow(measure.y/16.0, 2) + pow(measure.z/16.0, 2));
      return Vect3D(measure.x/scale, measure.y/scale, measure.z/scale);
    }
};

#endif

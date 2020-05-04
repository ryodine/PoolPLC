#ifndef ACCELEROMETER_MODEL_H
#define ACCELEROMETER_MODEL_H

#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

typedef struct {
  double x;
  double y;
  double z;
} AccelerometerModelZeropoint;


class AccelerometerModel {
  private:
    Matrix3d baseFrame;
    Vector3d zeroedForceVec;

  public:
    AccelerometerModel() : baseFrame(AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ())) {};
    // Config
    void importZero(AccelerometerModelZeropoint data);
    AccelerometerModelZeropoint setMeasurementAsZero(Vector3d measurement);
    void setBaseFrameAnglesRadians(Vector3d angles);

    Vector2d calculate(Vector3d measurement);
};


#endif

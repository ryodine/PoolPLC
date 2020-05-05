#ifndef ACCELEROMETER_MODEL_H
#define ACCELEROMETER_MODEL_H

#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

typedef struct {
  double m00;
  double m01;
  double m02;
  double m10;
  double m11;
  double m12;
  double m20;
  double m21;
  double m22;
} AccelerometerModelZeropoint;


class AccelerometerModel {
  private:
    Matrix3d baseFrame;
    Matrix3d zeroFrame;

    Matrix3d getFrameFromMeasurement(Vector3d measurement);

  public:
    AccelerometerModel() : baseFrame(AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ())),
      zeroFrame(AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ())) {};
    
    // Config
    void importZero(AccelerometerModelZeropoint data);
    AccelerometerModelZeropoint setMeasurementAsZero(Vector3d measurement);
    void setBaseFrameAnglesRadians(Vector3d angles);

    Vector2d calculate(Vector3d measurement);
};


#endif

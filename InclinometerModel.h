#ifndef INCLINOMETER_MODEL_H
#define INCLINOMETER_MODEL_H

#include <stlport.h>

#include <Eigen30.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace Eigen;

namespace Inclinometer {
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
} ModelZeropoint;

class Model {
  private:
    Matrix3d baseFrame;
    Matrix3d zeroFrame;

    Matrix3d convertAnglesToFrame(Vector2d angleMeasures);

  public:
    Model()
        : baseFrame(AngleAxisd(0, Vector3d::UnitX()) *
                    AngleAxisd(0, Vector3d::UnitY()) *
                    AngleAxisd(0, Vector3d::UnitZ())),
          zeroFrame(AngleAxisd(0, Vector3d::UnitX()) *
                    AngleAxisd(0, Vector3d::UnitY()) *
                    AngleAxisd(0, Vector3d::UnitZ())){};

    // Config
    void importZero(ModelZeropoint data);
    ModelZeropoint setMeasurementAsZero(Vector2d angleMeasures);
    void setBaseFrameAnglesRadians(Vector3d angles);

    Vector2d calculate(Vector2d angleMeasures);
};
}; // namespace Inclinometer

#endif

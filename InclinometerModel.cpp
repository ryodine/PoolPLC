#include "InclinometerModel.h"

using namespace Eigen;

void Inclinometer::Model::importZero(Inclinometer::ModelZeropoint data)
{
    Matrix3d m;
    // clang-format off
    m << data.m00, data.m01, data.m02,
         data.m10, data.m11, data.m12,
         data.m20, data.m21, data.m22;
    // clang-format on
    zeroFrame = m;
}

Inclinometer::ModelZeropoint
Inclinometer::Model::setMeasurementAsZero(Vector2d angleMeasures)
{
    Matrix3d mF = convertAnglesToFrame(angleMeasures);
    Inclinometer::ModelZeropoint pt;
    pt.m00 = mF(0, 0);
    pt.m01 = mF(0, 1);
    pt.m02 = mF(0, 2);
    pt.m10 = mF(1, 0);
    pt.m11 = mF(1, 1);
    pt.m12 = mF(1, 2);
    pt.m20 = mF(2, 0);
    pt.m21 = mF(2, 1);
    pt.m22 = mF(2, 2);
    importZero(pt);
    return pt;
}

void Inclinometer::Model::setBaseFrameAnglesRadians(Vector3d angles)
{
    Matrix3d rotMat;
    rotMat = AngleAxisd(angles[0], Vector3d::UnitX()) *
             AngleAxisd(angles[1], Vector3d::UnitY()) *
             AngleAxisd(angles[2], Vector3d::UnitZ());
    this->baseFrame = rotMat;
}

Vector2d Inclinometer::Model::calculate(Vector2d angleMeasures)
{
    Matrix3d measuredFrame = convertAnglesToFrame(angleMeasures);
    Vector3d angles = (this->baseFrame * zeroFrame.transpose() * measuredFrame)
                          .eulerAngles(0, 1, 2);
    return Vector2d(angles[1], angles[0]);
}

Matrix3d Inclinometer::Model::convertAnglesToFrame(Vector2d angleMeasures)
{
    Matrix3d measuredFrame;
    measuredFrame = AngleAxisd(angleMeasures[0], Vector3d::UnitX()) *
                    AngleAxisd(angleMeasures[1], Vector3d::UnitY()) *
                    AngleAxisd(0, Vector3d::UnitZ());
    return measuredFrame;
}

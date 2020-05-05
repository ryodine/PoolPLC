#include "AccelerometerModel.h"

using namespace Eigen;

void AccelerometerModel::importZero(AccelerometerModelZeropoint data)
{
  Matrix3d m;
  m << data.m00, data.m01, data.m02,
       data.m10, data.m11, data.m12,
       data.m20, data.m21, data.m22;

  zeroFrame = m;
}

AccelerometerModelZeropoint AccelerometerModel::setMeasurementAsZero(Vector3d measurement)
{
  Matrix3d mF = getFrameFromMeasurement(measurement);
  AccelerometerModelZeropoint pt;
  pt.m00 = mF(0,0);
  pt.m01 = mF(0,1);
  pt.m02 = mF(0,2);
  pt.m10 = mF(1,0);
  pt.m11 = mF(1,1);
  pt.m12 = mF(1,2);
  pt.m20 = mF(2,0);
  pt.m21 = mF(2,1);
  pt.m22 = mF(2,2);
  importZero(pt);
  return pt;
}

void AccelerometerModel::setBaseFrameAnglesRadians(Vector3d angles)
{
  Matrix3d rotMat;
  rotMat = AngleAxisd(angles[0], Vector3d::UnitX()) * AngleAxisd(angles[1], Vector3d::UnitY()) * AngleAxisd(angles[2], Vector3d::UnitZ());
  this->baseFrame = rotMat;
}

Vector2d AccelerometerModel::calculate(Vector3d measure)
{
  Matrix3d measuredFrame = getFrameFromMeasurement(measure);
  Vector3d angles = (this->baseFrame * zeroFrame.transpose() * measuredFrame).eulerAngles(0, 1, 2);
  return Vector2d(angles[1], angles[0]);
}

Matrix3d AccelerometerModel::getFrameFromMeasurement(Vector3d measure)
{
  double scale = sqrt(pow(measure[0]/16.0, 2) + pow(measure[1]/16.0, 2) + pow(measure[2]/16.0, 2));
  Vector3d normalized = Vector3d(measure[0]/scale, measure[1]/scale, measure[2]/scale);
  
  double pitch = atan(normalized[1]/normalized[2]);
  double roll = atan((-normalized[0])/sqrt(pow(normalized[1], 2)+pow(normalized[2], 2)));

  Matrix3d measuredFrame;
  measuredFrame = AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
  return measuredFrame;
}

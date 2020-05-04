#include "AccelerometerModel.h"

using namespace Eigen;

void AccelerometerModel::importZero(AccelerometerModelZeropoint data)
{
  zeroedForceVec = Vector3d(data.x, data.y, data.z);
}

AccelerometerModelZeropoint AccelerometerModel::setMeasurementAsZero(Vector3d measurement)
{
  AccelerometerModelZeropoint pt;
  pt.x = measurement[0];
  pt.y = measurement[1];
  pt.z = measurement[2];
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
  double scale = sqrt(pow(measure[0]/16.0, 2) + pow(measure[1]/16.0, 2) + pow(measure[2]/16.0, 2));
  Vector3d normalized = Vector3d(measure[0]/scale, measure[1]/scale, measure[2]/scale);
  
  double pitch = atan(normalized[1]/normalized[2]);
  double roll = atan((-normalized[0])/sqrt(pow(normalized[1], 2)+pow(normalized[2], 2)));

  Matrix3d measuredFrame;
  measuredFrame = AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());

  Vector3d angles = (this->baseFrame * measuredFrame).eulerAngles(0, 1, 2);

  return Vector2d(angles[1], angles[0]);
}

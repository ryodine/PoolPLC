#ifndef INCLINOMETER_MODULE_H
#define INCLINOMETER_MODULE_H

#include "InclinometerInterface.h"

namespace Inclinometer {
class Module {
  public:
    Module(InclinometerDataSource *src, double yaw = 0.0) : sensor(src)
    {
        model.setBaseFrameAnglesRadians(Vector3d(0, 0, yaw));
    };
    bool begin() { return sensor->begin(); };
    bool hasData() { return sensor->hasData(); };
    Eigen::Vector2d getData() { return model.calculate(sensor->getData()); };
    ModelZeropoint zero()
    {
        return model.setMeasurementAsZero(sensor->getData());
    };
    void importZero(ModelZeropoint zero) { return model.importZero(zero); };

  private:
    InclinometerDataSource *sensor;
    Model model;
};
}; // namespace Inclinometer

#endif
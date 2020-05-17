#ifndef PLC_INCLINOMETER_INTERFACE_H
#define PLC_INCLINOMETER_INTERFACE_H
#include <stlport.h>

#include <Eigen30.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Inclinometer {
class InclinometerDataSource {
  public:
    virtual bool begin();
    virtual bool hasData();
    virtual Eigen::Vector2d getData();
};
} // namespace Inclinometer

#endif
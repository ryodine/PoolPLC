/**
 * Interface for the ACEINNA Inclinometer over the CAN Bus
 */

#ifndef ACEINNA_MTLT_INCINOMETER_CAN_INTERFACE_H
#define ACEINNA_MTLT_INCINOMETER_CAN_INTERFACE_H

#include "CANSAEJ1939.h"
#include "InclinometerInterface.h"

#include <Arduino.h>

namespace Inclinometer {
class ACEINNAInclinometer : public InclinometerDataSource {
  public:
    ACEINNAInclinometer(HardwareSerial &canSerialInterface)
        : canInterface(canSerialInterface)
    {
    }

    bool begin() override;
    bool hasData() override;
    Eigen::Vector2d getData() override;

  //private:
    CAN::J1939Interface canInterface;
};
}; // namespace Inclinometer

#endif
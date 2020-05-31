/**
 * @file ACEINNAInclinometer.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Defines a high-level interface to the ACEINNA MTLT Industrial
 * Inclinometer
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef ACEINNA_MTLT_INCINOMETER_CAN_INTERFACE_H
#define ACEINNA_MTLT_INCINOMETER_CAN_INTERFACE_H

#include "CANSAEJ1939.h"
#include "InclinometerInterface.h"
#include "MovingAverage.h"

#include <Arduino.h>

namespace Inclinometer {

/**
 * @brief An interface to an ACEINNA MTLT Inclinometer
 */
class ACEINNAInclinometer : public InclinometerDataSource {
  public:
    /**
     * @brief Construct a new ACEINNAInclinometer object
     *
     * @param canSerialInterface the HardwareSerial interface on the Arduino
     * that is connected to the CAN Bus Serial Module
     * @param ewmaAlpha the alpha for exponentially weighted moving average
     * smoothing. By default, no smoothing happens (alpha=1)
     */
    ACEINNAInclinometer(HardwareSerial &canSerialInterface,
                        float ewmaAlpha = 1.0)
        : canInterface(canSerialInterface), hasDataCached(false),
          roll(0, ewmaAlpha), pitch(0, ewmaAlpha)
    {
    }

    //! ACEINNA CAN Bus Address
    static constexpr byte k_aceinnaAddress = 0x80;

    //! Address to simulate with this controller
    static constexpr byte k_sourceAddress = 0x11;

    //! Allowed angle range before considering the inclinometer data to be
    //! invalid (degrees)
    static constexpr double k_anglePlausibilityRange = 5;

        /**
         * @brief PGNs used by this module
         */
        enum PGN {
            // Data Messages / Get Messages
            PGN_ENABLED_PERIODIC_DATA_TYPES = 61366,
            PGN_SSI2DATA = 61481,

            // Command Messages
            PGN_SAVE_EEPROM = 65361,
            PGN_ODR = 65365,
            PGN_PERIODIC_DATA_TYPES,
            PGN_LOW_PASS,
            PGN_ORIENTATION
        };

    //! See the InclinometerDataSource interface

    bool begin() override;
    bool hasData() override;
    Eigen::Vector2d getData() override;

  private:
    CAN::J1939Interface canInterface;

    MovingAverage roll;
    MovingAverage pitch;

    Eigen::Vector2d cachedAngles;
    bool hasDataCached;
};
}; // namespace Inclinometer

#endif
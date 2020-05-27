/**
 * @file InclinometerInterface.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Describes the API to an Inclinometer Class
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PLC_INCLINOMETER_INTERFACE_H
#define PLC_INCLINOMETER_INTERFACE_H
#include <stlport.h>

#include <Eigen30.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Inclinometer {
/**
 * @brief Interface for reading from an inclinometer
 */
class InclinometerDataSource {
  public:
    /**
     * @brief Initializes the inclinometer
     *
     * @return true if the inclinometer successfully initializes
     * @return false if the inclinometer failed to initialize
     */
    virtual bool begin();

    /**
     * @brief Checks if the inclinometer has new data available
     *
     * @return true if there is data available
     * @return false if there is not data available
     */
    virtual bool hasData();

    /**
     * @brief Get the 2D vector of euler angles in radians (pitch, roll)
     *
     * @return Eigen::Vector2d Pitch and Roll, in radians
     */
    virtual Eigen::Vector2d getData();
};
} // namespace Inclinometer

#endif
/**
 * @file InclinometerModule.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief A module that encapsulates the mathematical model and the sensor for
 * an Inclinometer.
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef INCLINOMETER_MODULE_H
#define INCLINOMETER_MODULE_H

#include "InclinometerInterface.h"

namespace Inclinometer {

/**
 * @brief Holds a model and a data source
 */
class Module {
  public:

    /**
     * @brief Construct a new Module object
     * 
     * @param src the sensor data source
     * @param yaw the starting yaw offset, default 0
     */
    Module(InclinometerDataSource *src, double yaw = 0.0) : sensor(src)
    {
        model.setBaseFrameAnglesRadians(Vector3d(0, 0, yaw));
    };

    /**
     * @brief Initializes the contained sensor/data source
     * 
     * @return true if the sensor successfully initialized
     * @return false if the sensor did not successfully initialize
     */
    bool begin() { return sensor->begin(); };

    /**
     * @brief Checks if the sensor has data
     * 
     * @return true if the sensor has data that can be read
     * @return false if the sensor does not have data that can be read
     */
    bool hasData() { return sensor->hasData(); };

    /**
     * @brief Get the calculated angle measures if data is available
     * 
     * @return Eigen::Vector2d roll, pitch
     */
    Eigen::Vector2d getData() { return model.calculate(sensor->getData()); };

    /**
     * @brief zero the sensor and return the zero frame from the current measurement
     * 
     * @return ModelZeropoint 
     */
    ModelZeropoint zero()
    {
        return model.setMeasurementAsZero(sensor->getData());
    };

    /**
     * @brief import a zero frame and zero the sensor to it
     * 
     * @param zero the zero frame to zero the sensor to
     */
    void importZero(ModelZeropoint zero) { return model.importZero(zero); };

  private:
    InclinometerDataSource *sensor;
    Model model;
};
}; // namespace Inclinometer

#endif
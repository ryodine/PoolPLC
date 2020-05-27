/**
 * @file InclinometerModel.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Mathematical Model of the Inclinometer
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef INCLINOMETER_MODEL_H
#define INCLINOMETER_MODEL_H

#include <stlport.h>

#include <Eigen30.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

namespace Inclinometer {

/**
 * @brief The flattened version of a eigen matrix3d, which represents a
 * rotation. This "frame" can be stored to represent the zero pose of an
 * inclinometer.
 *
 * This model uses Roll-Pitch-Yaw euler rotations (Left-To-Right), or
 * Yaw-Pitch-Roll (Right-to-Left)
 */
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

/**
 * @brief The Inclinometer Model
 *
 * This model allows a pitch-roll inclinometer to have a yaw offset and a
 * programmable zero pose.
 */
class Model {
  public:
    /**
     * @brief Construct a new Model object
     */
    Model()
        : baseFrame(AngleAxisd(0, Vector3d::UnitX()) *
                    AngleAxisd(0, Vector3d::UnitY()) *
                    AngleAxisd(0, Vector3d::UnitZ())),
          zeroFrame(AngleAxisd(0, Vector3d::UnitX()) *
                    AngleAxisd(0, Vector3d::UnitY()) *
                    AngleAxisd(0, Vector3d::UnitZ())){};

    /**
     * @brief Imports a ModelZeropoint and updates the model
     *
     * @param data the zero point to import
     */
    void importZero(ModelZeropoint data);

    /**
     * @brief Zeroes the model based on the passed in measurement, and returns
     * the ModelZeropoint that can be used to come back to this zero point.
     *
     * @param angleMeasures angle measures (inclinometer pitch/roll)
     * @return ModelZeropoint
     */
    ModelZeropoint setMeasurementAsZero(Vector2d angleMeasures);

    /**
     * @brief Set the base frame (static rotation offsets) from three euler
     * angles (roll, pitch, and yaw)
     *
     * @param angles roll, pitch, yaw
     */
    void setBaseFrameAnglesRadians(Vector3d angles);

    /**
     * @brief Apply zero and base frame to get new coordinates.
     * 
     * @param angleMeasures input (measured) angles (roll, pitch)
     * @return Vector2d output (calculated) angles (roll, pitch)
     */
    Vector2d calculate(Vector2d angleMeasures);

  private:
    Matrix3d baseFrame;
    Matrix3d zeroFrame;

    Matrix3d convertAnglesToFrame(Vector2d angleMeasures);
};
}; // namespace Inclinometer

#endif

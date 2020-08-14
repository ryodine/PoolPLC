/**
 * @file MotionController.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief This class controls the movement of the platform based on the state
 * machine that is uses
 * @version 0.1
 * @date 2020-05-30
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef MOTION_CONTROLLER_GUARD_H
#define MOTION_CONTROLLER_GUARD_H

#include "Constants.h"
#include "DisplayControl.h"
#include "HighestCornerAlgorithm.h"
#include "InclinometerModule.h"
#include "MotionStateMachine.h"

#include <stlport.h>

#include <Eigen30.h>

#include <Controllino.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace {
constexpr int k_dispUpdatePeriodMillis = 1000;

/**
 * @brief The hydraulic rams in the highest corner algorithm use a different
 * numbering convention than the convention used onsite. This map transalates
 * the conventions.
 *
 * @details THe highest corner algorithm's convention is to use the quadrant
 * number of a 2d coordinate plane as the ram number. The quadrant is defined by
 * sensor pitch representing the y-axis, and sensor roll representing the
 * x-axis.
 */
enum class CORNER_REMAPPER {
    RAM_1_RAISE = PIN_CAST(Constants::Pins::RAM::RAISE_2),
    RAM_1_LOWER = PIN_CAST(Constants::Pins::RAM::LOWER_2),
    RAM_2_RAISE = PIN_CAST(Constants::Pins::RAM::RAISE_3),
    RAM_2_LOWER = PIN_CAST(Constants::Pins::RAM::LOWER_3),
    RAM_3_RAISE = PIN_CAST(Constants::Pins::RAM::RAISE_4),
    RAM_3_LOWER = PIN_CAST(Constants::Pins::RAM::LOWER_4),
    RAM_4_RAISE = PIN_CAST(Constants::Pins::RAM::RAISE_1),
    RAM_4_LOWER = PIN_CAST(Constants::Pins::RAM::LOWER_1)
};
//! "logical" version of the above remapping, for the debug display. The above
//! version is a remapping of actual outputs
enum class CORNER_REMAPPER_LOGICAL {
    RAM_1 = 3,
    RAM_2 = 0,
    RAM_3 = 1,
    RAM_4 = 2
};
} // namespace

namespace Motion {
class MotionController {
  public:
    MotionController(Inclinometer::Module &sensor);

    /**
     * @brief Set up the motion controller
     *
     * @return true set up worked
     * @return false failed to set up
     */
    bool Initialize();

    /**
     * @brief Requests that the controller transition to raising mode
     */
    void RequestRaise();

    /**
     * @brief Requests that the controller transition to lowering mode
     */
    void RequestLower();

    /**
     * @brief Requests that the controller halts motion
     */
    void RequestOff();

    /**
     * @brief Requests that the controller transition to lowering mode
     */
    void RequestClearFaultState();

    Eigen::Vector2d GetLastMeasures() { return m_lastSensorMeasures; };

    /**
     * @brief Steps the controller and state machine (call this iteratively)
     */
    void Step();

    /**
     * @brief Get the State of the state machine
     *
     * @return MotionStateMachine::STATE current state of the state machine
     */
    MotionStateMachine::STATE GetState() { return m_stateMachine.GetState(); }

    /**
     * @brief Get the Direction of movement
     *
     * @return MovementDirection
     */
    MovementDirection GetDirection() { return m_direction; }

    /**
     * @brief Shows a brief message on screen, will disappear on next update of
     * motion controller
     *
     * @param line2 text to show on line 2
     */
    void PopMessage(char *line2);

  private:
    Inclinometer::Module &m_sensor;
    MotionStateMachine m_stateMachine;
    Display::Controller m_displayController;
    unsigned long m_lastDispUpdate;

    HighestCornerAlgo m_cornerAlgo;

    MovementDirection m_direction = NONE;

    unsigned long m_lastSensorReadingTimestamp;
    unsigned long m_lastSensorReadingUnstable;
    Eigen::Vector2d m_lastSensorMeasures;

    // Callback hooks from state machine:
    void StartMovement();
    void StopMovement();
    void SetCorners(bool corner1, bool corner2, bool corner3, bool corner4,
                    bool raising);
    void MovementAlgorithmStep();
    bool CheckStabilityStep();

    // Disp update Step
    void DispUpdate();

    // Let the state machine access this class' private functions
    friend class MotionStateMachine;
};

} // namespace Motion

#endif // MOTION_CONTROLLER_GUARD_H
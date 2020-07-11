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

#include "HighestCornerAlgorithm.h"
#include "InclinometerModule.h"
#include "MotionStateMachine.h"
#include "PinMappings.h"
#include "DisplayControl.h"

#include <stlport.h>

#include <Eigen30.h>

#include <Controllino.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Motion {

constexpr int k_dispUpdatePeriodMillis = 1000;

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
     * @brief Shows a brief message on screen, will disappear on next update of motion controller
     * 
     * @param line2 text to show on line 2
     */
    void PopMessage(char* line2);

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
    void SetCorners(bool corner1, bool corner2, bool corner3, bool corner4, bool raising);
    void MovementAlgorithmStep();
    bool CheckStabilityStep();

    // Disp update Step
    void DispUpdate();

    // Let the state machine access this class' private functions
    friend class MotionStateMachine;
};

} // namespace Motion

#endif // MOTION_CONTROLLER_GUARD_H
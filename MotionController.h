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

#include <stlport.h>

#include <Eigen30.h>

#include <Controllino.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


#define OUT_TR CONTROLLINO_D16
#define OUT_TL CONTROLLINO_D17
#define OUT_BL CONTROLLINO_D18
#define OUT_BR CONTROLLINO_D19

#define OUT_ENABLE CONTROLLINO_D23

namespace Motion {

enum MovementDirection { RAISE, LOWER, NONE };

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

    void RequestRaise();

    void RequestLower();

    void RequestOff();

    Eigen::Vector2d GetLastMeasures() { return m_lastSensorMeasures; };

    /**
     * @brief Steps the controller and state machine (call this iteratively)
     */
    void Step();

  private:
    Inclinometer::Module &m_sensor;
    MotionStateMachine m_stateMachine;

    HighestCornerAlgo m_cornerAlgo;

    MovementDirection m_direction = NONE;

    unsigned long m_lastSensorReadingTimestamp;
    unsigned long m_lastSensorReadingUnstable;
    Eigen::Vector2d m_lastSensorMeasures;

    // Called from state machine:
    void StartMovement();
    void StopMovement();
    void SetCorners(bool corner1, bool corner2, bool corner3, bool corner4);
    void MovementAlgorithmStep();
    bool CheckStabilityStep();

    // Let the state machine access this class' private functions
    friend class MotionStateMachine;
};

} // namespace Motion

#endif // MOTION_CONTROLLER_GUARD_H
/**
 * @file MotionStateMachine.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief This class is a state machine that enforces preconditions and running
 * conditions of allowing the platform to move
 * @version 0.1
 * @date 2020-05-30
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef MOVING_STATE_MACHINE_GUARD_H
#define MOVING_STATE_MACHINE_GUARD_H

namespace Motion {

class MotionController;

class MotionStateMachine {
  public:
    enum STATE {
        STATE_NONE,
        STATE_NOT_RUNNING,
        STATE_MOVEMENT_REQUESTED,
        STATE_MOVING
    };

    MotionStateMachine(MotionController *controller);

    void Step();
    void RequestState(STATE state) { m_requestedState = state; };
    STATE GetState() { return m_currentState; };

  private:
    void OnStateNotRunningEnter();
    STATE OnStateNotRunningStep();
    void OnStateNotRunningExit();

    void OnStateMovementRequestedEnter();
    STATE OnStateMovementRequestedStep();
    void OnStateMovementRequestedExit();

    void OnStateMovingEnter();
    STATE OnStateMovingStep();
    void OnStateMovingExit();

    MotionController *m_controller;
    STATE m_currentState;
    STATE m_requestedState;
    unsigned long m_stateStartMillis;
};

} // namespace Motion

#endif // MOVING_STATE_MACHINE_GUARD_H
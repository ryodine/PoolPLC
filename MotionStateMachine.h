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

#ifndef ENUM_TO_STRING
#define __ENUM_TO_STRING__(x) #x
#define ENUM_TO_STRING(x) __ENUM_TO_STRING__(x)
#endif

namespace Motion {

enum MovementDirection { RAISE, LOWER, NONE };

class MotionController;
constexpr char* k_motionStateNames[] = {"READY", "STOPPED", "STEADYING", "MOVING", "FAULTED"};


class MotionStateMachine {
  public:
    enum STATE {
        STATE_NONE,
        STATE_NOT_RUNNING,
        STATE_MOVEMENT_REQUESTED,
        STATE_MOVING,
        STATE_FAULTED
    };

    MotionStateMachine(MotionController *controller);

    //! Steps the state machine
    void Step();

    //! Perform a transition from current state to "state" on next step
    void RequestState(STATE state) { m_requestedState = state; };

    //! Gets the current state
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

    void OnStateFaultedEnter();
    STATE OnStateFaultedStep();
    void OnStateFaultedExit();

    MotionController *m_controller;
    STATE m_currentState;
    STATE m_requestedState;
    unsigned long m_stateStartMillis;
};

} // namespace Motion

#endif // MOVING_STATE_MACHINE_GUARD_H
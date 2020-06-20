#include "MotionStateMachine.h"

#include "MotionController.h"

#include <Arduino.h>

Motion::MotionStateMachine::MotionStateMachine(MotionController *controller)
    : m_controller(controller), m_currentState(STATE_NONE),
      m_requestedState(STATE_NONE)
{
}

void Motion::MotionStateMachine::Step()
{
    if (m_currentState != m_requestedState) {
        if (m_currentState == Motion::MotionStateMachine::STATE_FAULTED &&
            m_requestedState != Motion::MotionStateMachine::STATE_NOT_RUNNING) {
            // Ignore transitions in fault that aren't stop
            return;
        }

        switch (m_currentState) {
        case Motion::MotionStateMachine::STATE_NOT_RUNNING:
            OnStateNotRunningExit();
            break;
        case Motion::MotionStateMachine::STATE_MOVEMENT_REQUESTED:
            OnStateMovementRequestedExit();
            break;
        case Motion::MotionStateMachine::STATE_MOVING:
            OnStateMovingExit();
            break;
        case Motion::MotionStateMachine::STATE_FAULTED:
            OnStateFaultedExit();
            break;
        default:
            break;
        }

        switch (m_requestedState) {
        case Motion::MotionStateMachine::STATE_NOT_RUNNING:
            OnStateNotRunningEnter();
            break;
        case Motion::MotionStateMachine::STATE_MOVEMENT_REQUESTED:
            OnStateMovementRequestedEnter();
            break;
        case Motion::MotionStateMachine::STATE_MOVING:
            OnStateMovingEnter();
            break;
        case Motion::MotionStateMachine::STATE_FAULTED:
            OnStateFaultedEnter();
            break;
        default:
            break;
        }

        m_currentState = m_requestedState;
        m_stateStartMillis = millis();
    }

    switch (m_currentState) {
    case Motion::MotionStateMachine::STATE_NOT_RUNNING:
        m_requestedState = OnStateNotRunningStep();
        break;
    case Motion::MotionStateMachine::STATE_MOVEMENT_REQUESTED:
        m_requestedState = OnStateMovementRequestedStep();
        break;
    case Motion::MotionStateMachine::STATE_MOVING:
        m_requestedState = OnStateMovingStep();
        break;
    case Motion::MotionStateMachine::STATE_FAULTED:
        m_requestedState = OnStateFaultedStep();
        break;
    default:
        break;
    }
}

void Motion::MotionStateMachine::OnStateNotRunningEnter()
{
    m_controller->StopMovement();
}
Motion::MotionStateMachine::STATE
Motion::MotionStateMachine::OnStateNotRunningStep()
{
    // If there is a fault, transition to no movement
    if (Fault::Handler::instance()->hasFault()) {
        return Motion::MotionStateMachine::STATE_FAULTED;
    }
    return STATE_NOT_RUNNING;
}
void Motion::MotionStateMachine::OnStateNotRunningExit()
{
    // no - op
}

void Motion::MotionStateMachine::OnStateMovementRequestedEnter()
{
    Serial.println("Waiting for stabilization");
}
Motion::MotionStateMachine::STATE
Motion::MotionStateMachine::OnStateMovementRequestedStep()
{
    // If there is a fault, transition to no movement
    if (Fault::Handler::instance()->hasFault()) {
        return Motion::MotionStateMachine::STATE_FAULTED;
    }

    // It is ok to move if stability has been reached once
    if (m_controller->CheckStabilityStep())
        return Motion::MotionStateMachine::STATE_MOVING;
    return Motion::MotionStateMachine::STATE_MOVEMENT_REQUESTED;
}

void Motion::MotionStateMachine::OnStateMovementRequestedExit()
{
    Serial.println("Stabilization target reached");
}

void Motion::MotionStateMachine::OnStateMovingEnter()
{
    m_controller->StartMovement();
}

Motion::MotionStateMachine::STATE
Motion::MotionStateMachine::OnStateMovingStep()
{
    // If there is a fault, transition to no movement
    if (Fault::Handler::instance()->hasFault()) {
        return Motion::MotionStateMachine::STATE_FAULTED;
    }
    m_controller->MovementAlgorithmStep();
    return Motion::MotionStateMachine::STATE_MOVING;
}
void Motion::MotionStateMachine::OnStateMovingExit()
{
    m_controller->StopMovement();
}

void Motion::MotionStateMachine::OnStateFaultedEnter()
{
    Serial.println("Fault state entered");
}

Motion::MotionStateMachine::STATE
Motion::MotionStateMachine::OnStateFaultedStep()
{
    return Motion::MotionStateMachine::STATE_FAULTED;
}
void Motion::MotionStateMachine::OnStateFaultedExit()
{
    //
}
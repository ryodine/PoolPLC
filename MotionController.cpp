#include "MotionController.h"

#include "FaultHandling.h"
#include "InclinometerModel.h"

#include <Arduino.h>

Motion::MotionController::MotionController(Inclinometer::Module& sensor)
    : m_sensor(sensor), m_stateMachine(MotionStateMachine(this)),
      m_cornerAlgo(0.05 / 180.0 * PI, 0.1 / 180.0 * PI)
{
}

bool Motion::MotionController::Initialize()
{
    pinMode(OUT_TR, OUTPUT);
    pinMode(OUT_BR, OUTPUT);
    pinMode(OUT_TL, OUTPUT);
    pinMode(OUT_BL, OUTPUT);
    pinMode(OUT_ENABLE, OUTPUT);

    SetCorners(false, false, false, false);
    digitalWrite(OUT_ENABLE, LOW);
    m_lastSensorReadingTimestamp = millis();
    return true;
}

void Motion::MotionController::RequestRaise()
{
    m_stateMachine.RequestState(MotionStateMachine::STATE_MOVEMENT_REQUESTED);
    m_direction = RAISE;
}

void Motion::MotionController::RequestLower()
{
    m_stateMachine.RequestState(MotionStateMachine::STATE_MOVEMENT_REQUESTED);
    m_direction = LOWER;
}

void Motion::MotionController::RequestOff()
{
    m_stateMachine.RequestState(MotionStateMachine::STATE_NOT_RUNNING);
    m_direction = NONE;
}

void Motion::MotionController::Step()
{
    if (m_sensor.hasData()) {

        // If the inclinometer has data ready, then we can safely unlatch &
        // reset the no data ready fault
        Fault::Handler::instance()->onFaultUnlatchEvent(
            Fault::FaultUnlatchEvent::INCLINOMETER_DATA_RECEIVE);
        m_lastSensorReadingTimestamp = millis();

        // Calculate the angles from the data using the mathematical model
        m_lastSensorMeasures = m_sensor.getData();

        double senseRollRate =
            m_sensor.getModel().getAngularAveragedVelocities()[0] * 18000.0 / PI;
        double sensePitchRate =
            m_sensor.getModel().getAngularAveragedVelocities()[1] * 18000.0 / PI;

        if (senseRollRate >= 0.1 || sensePitchRate >= 0.1) {
            m_lastSensorReadingUnstable = millis();
        }

        Serial.print(m_lastSensorMeasures[0] * 180.0 / PI);
        Serial.print("\t");
        Serial.print(m_lastSensorMeasures[1] * 180.0 / PI);
        Serial.print("\t");
        Serial.print(senseRollRate);
        Serial.print("\t");
        Serial.print(sensePitchRate);
        Serial.print("\n");
    }

    if (millis() - m_lastSensorReadingTimestamp > 500) {
        Fault::Handler::instance()->setFaultCode(Fault::INCLINOMETER_NOT_READY);
    }

    // If there is a fault, transition to no movement
    if (Fault::Handler::instance()->hasFault()) {
        m_stateMachine.RequestState(MotionStateMachine::STATE_NOT_RUNNING);
    }
    m_stateMachine.Step();
}

void Motion::MotionController::StartMovement()
{
    // to-do start movement
    Serial.println("Movement started");
    digitalWrite(OUT_ENABLE, HIGH);
}

void Motion::MotionController::StopMovement()
{
    // to-do stop movement
    Serial.println("Movement halted");
    SetCorners(false, false, false, false);
    digitalWrite(OUT_ENABLE, LOW);
    Fault::Handler::instance()->onFaultUnlatchEvent(
        Fault::FaultUnlatchEvent::MOVEMENT_COMMAND_END);
}

void Motion::MotionController::SetCorners(bool corner1, bool corner2,
                                          bool corner3, bool corner4)
{
    // to-do set corners
    digitalWrite(OUT_TR, corner1);
    digitalWrite(OUT_TL, corner2);
    digitalWrite(OUT_BL, corner3);
    digitalWrite(OUT_BR, corner4);
}

void Motion::MotionController::MovementAlgorithmStep()
{
    m_cornerAlgo.update(m_lastSensorMeasures[0], m_lastSensorMeasures[1]);
    bool lowering = m_direction == LOWER;

    // Control the solenoids, if no faults and in raise or lower mode
    SetCorners(m_cornerAlgo.getCorner(0, lowering),
               m_cornerAlgo.getCorner(1, lowering),
               m_cornerAlgo.getCorner(2, lowering),
               m_cornerAlgo.getCorner(3, lowering));
}

bool Motion::MotionController::CheckStabilityStep()
{
    return (millis() - m_lastSensorReadingUnstable > 1000);
}
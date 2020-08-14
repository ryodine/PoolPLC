#include "MotionController.h"

#include "DisplayControl.h"
#include "FaultHandling.h"
#include "InclinometerModel.h"

#include <Arduino.h>

Motion::MotionController::MotionController(Inclinometer::Module &sensor)
    : m_sensor(sensor), m_stateMachine(MotionStateMachine(this)),
      m_cornerAlgo(Constants::Algorithm::k_stopCorrectingTiltAtDegrees / 180.0 *
                       PI,
                   Constants::Algorithm::k_correctTiltAtDegrees / 180.0 * PI)
{
}

bool Motion::MotionController::Initialize()
{
    pinMode(PIN_CAST(Constants::Pins::RAM::RAISE_1), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::RAISE_2), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::RAISE_3), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::RAISE_4), OUTPUT);

    pinMode(PIN_CAST(Constants::Pins::RAM::LOWER_1), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::LOWER_2), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::LOWER_3), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::RAM::LOWER_4), OUTPUT);

    pinMode(PIN_CAST(Constants::Pins::MOTOR::ENABLE_RAISE), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::MOTOR::ENABLE_LOWER), OUTPUT);

    // clears all ram disable pins
    SetCorners(false, false, false, false, false);
    SetCorners(false, false, false, false, true);

    // disable the output
    digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_RAISE), LOW);
    digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_LOWER), LOW);

    m_lastSensorReadingTimestamp = millis();
    m_lastDispUpdate = millis() - k_dispUpdatePeriodMillis;
    return m_displayController.begin();
}

void Motion::MotionController::RequestRaise()
{
    if (GetState() != MotionStateMachine::STATE_FAULTED) {
        m_stateMachine.RequestState(
            MotionStateMachine::STATE_MOVEMENT_REQUESTED);
        m_direction = RAISE;
    }
}

void Motion::MotionController::RequestLower()
{
    if (GetState() != MotionStateMachine::STATE_FAULTED) {
        m_stateMachine.RequestState(
            MotionStateMachine::STATE_MOVEMENT_REQUESTED);
        m_direction = LOWER;
    }
}

void Motion::MotionController::RequestOff()
{
    m_stateMachine.RequestState(MotionStateMachine::STATE_NOT_RUNNING);
    m_direction = NONE;
}

void Motion::MotionController::RequestClearFaultState()
{
    if (GetState() == MotionStateMachine::STATE_FAULTED &&
        !Fault::Handler::instance()->hasFault()) {
        RequestOff();
    }
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
            m_sensor.getModel().getAngularAveragedVelocities()[0] * 18000.0 /
            PI;
        double sensePitchRate =
            m_sensor.getModel().getAngularAveragedVelocities()[1] * 18000.0 /
            PI;

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
        Fault::Handler::instance()->setFaultCode(Fault::INCLINOMETER_UNREADY);
    }

    DispUpdate();
    m_stateMachine.Step();
}

void Motion::MotionController::StartMovement()
{
    Serial.println("Movement started");
    switch (m_direction) {
    case RAISE:
        digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_RAISE), HIGH);
        break;
    case LOWER:
        digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_LOWER), HIGH);
        break;
    }
}

void Motion::MotionController::StopMovement()
{
    Serial.println("Movement halted");
    SetCorners(false, false, false, false, m_direction == RAISE);
    digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_RAISE), LOW);
    digitalWrite(PIN_CAST(Constants::Pins::MOTOR::ENABLE_LOWER), LOW);

    Fault::Handler::instance()->onFaultUnlatchEvent(
        Fault::FaultUnlatchEvent::MOVEMENT_COMMAND_END);
}

void Motion::MotionController::SetCorners(bool corner1, bool corner2,
                                          bool corner3, bool corner4,
                                          bool raising)
{
    if (raising) {
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_1_RAISE), corner1);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_2_RAISE), corner2);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_3_RAISE), corner3);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_4_RAISE), corner4);

        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_1_LOWER), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_2_LOWER), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_3_LOWER), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_4_LOWER), false);
    }
    else {
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_1_LOWER), corner1);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_2_LOWER), corner2);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_3_LOWER), corner3);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_4_LOWER), corner4);

        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_1_RAISE), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_2_RAISE), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_3_RAISE), false);
        digitalWrite(PIN_CAST(CORNER_REMAPPER::RAM_4_RAISE), false);
    }
}

void Motion::MotionController::MovementAlgorithmStep()
{
    m_cornerAlgo.update(m_lastSensorMeasures[0], m_lastSensorMeasures[1]);
    bool lowering = m_direction == LOWER;

    // Control the solenoids, if no faults and in raise or lower mode
    // The algorithm provides the deviating corners. We need to only enable the
    // movement on the "good" corners, so we invert the output
    SetCorners(!m_cornerAlgo.getCorner(0, lowering),
               !m_cornerAlgo.getCorner(1, lowering),
               !m_cornerAlgo.getCorner(2, lowering),
               !m_cornerAlgo.getCorner(3, lowering), !lowering);
}

void Motion::MotionController::PopMessage(char *line2)
{
    Display::DisplayableText t;
    for (int i = 0; i < 4; i++) {
        strncpy(t.array[i], "                    ", 21);
    }
    snprintf(t.line_struct.line2, 21, "%-20s", line2);
    m_displayController.writeRaw(t);
}

void Motion::MotionController::DispUpdate()
{
    // Step the display controller
    if (millis() - m_lastDispUpdate > k_dispUpdatePeriodMillis) {
        Display::SystemDisplayState dstate;
        dstate.motionState = GetState();
        dstate.pitch = m_sensor.getData()[1] * 180.0 / PI;
        dstate.roll = m_sensor.getData()[0] * 180.0 / PI;
        dstate.ram1 = m_cornerAlgo.getCorner(
            static_cast<unsigned int>(CORNER_REMAPPER_LOGICAL::RAM_1),
            m_direction == LOWER);
        dstate.ram2 = m_cornerAlgo.getCorner(
            static_cast<unsigned int>(CORNER_REMAPPER_LOGICAL::RAM_2),
            m_direction == LOWER);
        dstate.ram3 = m_cornerAlgo.getCorner(
            static_cast<unsigned int>(CORNER_REMAPPER_LOGICAL::RAM_3),
            m_direction == LOWER);
        dstate.ram4 = m_cornerAlgo.getCorner(
            static_cast<unsigned int>(CORNER_REMAPPER_LOGICAL::RAM_4),
            m_direction == LOWER);
        dstate.dirn = m_direction;
        dstate.enable = GetState() == MotionStateMachine::STATE_MOVING;
        if (Fault::Handler::instance()->hasFault()) {
            dstate.faultType =
                Fault::Handler::instance()->nextFault(Fault::ZERO);
        }
        else {
            dstate.faultType = Fault::ALL_OK;
        }
        m_displayController.update(dstate);
        m_lastDispUpdate = millis();
    }
}

bool Motion::MotionController::CheckStabilityStep()
{
    return (millis() - m_lastSensorReadingUnstable > 1000);
}
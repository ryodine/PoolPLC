#include "MotionController.h"

#include "DisplayControl.h"
#include "FaultHandling.h"
#include "InclinometerModel.h"

#include <Arduino.h>

Motion::MotionController::MotionController(Inclinometer::Module &sensor)
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
    m_lastDispUpdate = millis() - k_dispUpdatePeriodMillis;
    return m_displayController.begin();
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
    digitalWrite(OUT_ENABLE, HIGH);
}

void Motion::MotionController::StopMovement()
{
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

void Motion::MotionController::PopMessage(char* line2) {
    Display::DisplayableText t;
    for (int i = 0; i < 4; i++) {
        strncpy(t.array[i], "                    ", 21);
    }
    snprintf(t.line_struct.line2, 21, "%-20s", line2);
    m_displayController.writeRaw(t);
}

void Motion::MotionController::DispUpdate() {
    // Step the display controller
    if (millis() - m_lastDispUpdate > k_dispUpdatePeriodMillis) {
        Display::SystemDisplayState dstate;
        dstate.motionState = GetState();
        dstate.pitch = m_sensor.getData()[1] * 180.0 / PI;
        dstate.roll = m_sensor.getData()[0] * 180.0 / PI;
        dstate.ram1 = m_cornerAlgo.getCorner(0, m_direction == LOWER);
        dstate.ram2 = m_cornerAlgo.getCorner(1, m_direction == LOWER);
        dstate.ram3 = m_cornerAlgo.getCorner(2, m_direction == LOWER);
        dstate.ram4 = m_cornerAlgo.getCorner(3, m_direction == LOWER);
        dstate.dirn = m_direction;
        dstate.enable = GetState() == MotionStateMachine::STATE_MOVING;
        if (Fault::Handler::instance()->hasFault()) {
            dstate.faultType = Fault::Handler::instance()->nextFault(Fault::ZERO);
        } else {
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
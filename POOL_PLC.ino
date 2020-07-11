/**
 * @file POOL_PLC.ino
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Main program for the Pool PLC
 * @version 0.1
 * @date 2020-06-13
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ACEINNAInclinometer.h"
#include "FaultHandling.h"
#include "InclinometerModel.h"
#include "InclinometerModule.h"
#include "PersistentStorage.h"
#include "MotionController.h"
#include "PinMappings.h"

#include <stlport.h>

#include <Eigen30.h>

#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>
#include <Controllino.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <SPI.h>


Fault::Handler *faultHandler;
PersistentStorage::Manager storageManager;

Inclinometer::ACEINNAInclinometer aceinna(Serial2, 0.5);
Inclinometer::Module inclinometer1(&aceinna, 0.0);

Motion::MotionController motionController(inclinometer1);

bool raising = false;
bool lowering = false;

void setup()
{
    faultHandler = Fault::Handler::instance();
    // put your setup code here, to run once:
    pinMode(STATUS_PIN, OUTPUT);
    pinMode(FAULT_PIN, OUTPUT);
    pinMode(ZERO_BUTTON, INPUT);
    pinMode(REFLASH_ACEINNA, INPUT);
    pinMode(RAISE_BUTTON, INPUT);
    pinMode(LOWER_BUTTON, INPUT);
    digitalWrite(STATUS_PIN, LOW);

    SPI.begin();
    Serial.begin(9600);
    Serial.println("Starting up");
    if (!inclinometer1.begin()) {
        faultHandler->setFaultCode(Fault::INCLINOMETER_INIT);
    }
    Serial.println("Inclinometer began");
    if (!storageManager.begin()) {
        faultHandler->setFaultCode(Fault::FRAM_INIT);
    }
    Serial.println("Storage began");
    storageManager.readMap();
    inclinometer1.importZero(storageManager.getMap()->zeroFrame1);
    motionController.Initialize();
    motionController.Step(); // Step the motion controller once to show any active faults
    Serial.println("Initialized");
}

/**
 * @brief This function is the main periodic loop, which runs forever (until the
 * controller powers off or resets)
 */
void loop()
{

    // Determine if raising or lowering
    bool wasRaising = raising;
    bool wasLowering = lowering;
    raising = digitalRead(RAISE_BUTTON);
    lowering = digitalRead(LOWER_BUTTON);
    raising = raising && !lowering;
    lowering = lowering && !raising;

    if (!wasRaising && raising) {
        motionController.RequestRaise();
    }

    if (!wasLowering && lowering) {
        motionController.RequestLower();
    }

    if (!lowering && !raising && (wasLowering || wasRaising)) {
        motionController.RequestOff();
    }

    // Step the motion controller
    motionController.Step();
    status_flash(motionController.GetState());
    digitalWrite(FAULT_PIN, Fault::Handler::instance()->hasFault());

    // Check if the user wanted to zero the inclinometer
    if (digitalRead(ZERO_BUTTON) && !(raising || lowering)) {
        storageManager.getMap()->zeroFrame1 = inclinometer1.zero();
        storageManager.writeMap();
        motionController.PopMessage("RESET LEVEL SENSOR");
        delay(500);
    }

    // Check if the user wanted to reflash the aceinna module
    if (digitalRead(REFLASH_ACEINNA) && !(raising || lowering)) {
        aceinna.ProvisionACEINNAInclinometer();
        motionController.PopMessage("FLASHED SENSE EEPROM");
        delay(500);
    }

    // avoid thrashing the sensor too much
    delay(10);
}

bool flash = false;
unsigned long last_pulse_at = 0;
void status_flash(Motion::MotionStateMachine::STATE state)
{
    switch (state) {
        case Motion::MotionStateMachine::STATE_MOVING:
            digitalWrite(STATUS_PIN, HIGH);
            break;
        case Motion::MotionStateMachine::STATE_MOVEMENT_REQUESTED:
            if (millis() - last_pulse_at > 500) {
                flash = !flash;
                last_pulse_at = millis();
                digitalWrite(STATUS_PIN, flash);
            }
            break;
        case Motion::MotionStateMachine::STATE_FAULTED:
            if (millis() - last_pulse_at > 50) {
                flash = !flash;
                last_pulse_at = millis();
                digitalWrite(STATUS_PIN, flash);
            }
            break;
        default:
            digitalWrite(STATUS_PIN, LOW);
    }
}
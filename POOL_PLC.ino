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
#include "Constants.h"
#include "FaultHandling.h"
#include "InclinometerModel.h"
#include "InclinometerModule.h"
#include "MotionController.h"
#include "MotionStateMachine.h"
#include "PersistentStorage.h"

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

Inclinometer::ACEINNAInclinometer
    aceinna(Serial2, Constants::Algorithm::k_inclinometerEWMASmoothingAlpha);
Inclinometer::Module
    inclinometer1(&aceinna,
                  Constants::Physical::k_inclinometerInstalledYawAdjustment);

Motion::MotionController motionController(inclinometer1);

bool raising = false;
bool lowering = false;

void setup()
{
    //! Output and Input setup ===============
    //! some outputs are also configured
    //! in the motion controller
    pinMode(PIN_CAST(Constants::Pins::INDICATOR::FAULT_ACTIVE), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::INDICATOR::FAULT_CLEARABLE), OUTPUT);
    pinMode(PIN_CAST(Constants::Pins::INDICATOR::READY), OUTPUT);

    pinMode(PIN_CAST(Constants::Pins::BUTTON::ZERO), INPUT);
    pinMode(PIN_CAST(Constants::Pins::BUTTON::REFLASH_ACEINNA), INPUT);
    pinMode(PIN_CAST(Constants::Pins::BUTTON::RAISE), INPUT);
    pinMode(PIN_CAST(Constants::Pins::BUTTON::LOWER), INPUT);
    pinMode(PIN_CAST(Constants::Pins::BUTTON::CLEAR_FAULT), INPUT);

    //! Declarations ==========================
    faultHandler = Fault::Handler::instance();

    //! Initializations =======================

    // Turn off all indicators
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::FAULT_ACTIVE), LOW);
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::FAULT_CLEARABLE), LOW);
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::READY), LOW);

    // Begin debugging interface (Serial)
    Serial.begin(9600);
    Serial.println("Starting up");

    // Setup storage
    if (!storageManager.begin()) {
        faultHandler->setFaultCode(Fault::FRAM_INIT);
    }
    Serial.println("Storage began");

    // Setup inclinometer
    if (!inclinometer1.begin()) {
        faultHandler->setFaultCode(Fault::INCLINOMETER_INIT);
    }
    Serial.println("Inclinometer began");
    storageManager.readMap();
    inclinometer1.importZero(storageManager.getMap()->zeroFrame1);

    // Initialize the motion controller
    motionController.Initialize();
    motionController
        .Step(); // Step the motion controller once to show any active faults

    Serial.println("Initialized");
}

/**
 * @brief This function is the main periodic loop, which runs forever (until the
 * controller powers off or resets)
 */
void loop()
{
    // Determine if raising or lowering
    bool wasRaising = motionController.GetDirection() == Motion::RAISE;
    bool wasLowering = motionController.GetDirection() == Motion::LOWER;
    raising = digitalRead(PIN_CAST(Constants::Pins::BUTTON::RAISE));
    lowering = digitalRead(PIN_CAST(Constants::Pins::BUTTON::LOWER));
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

    // Update indicators
    indicator_step(motionController.GetState());

    // Check if the user wanted to clear faults
    if (digitalRead(PIN_CAST(Constants::Pins::BUTTON::CLEAR_FAULT))) {
        motionController.RequestClearFaultState();
    }

    // Check if the user wanted to reflash the aceinna module
    if (digitalRead(PIN_CAST(Constants::Pins::BUTTON::REFLASH_ACEINNA))) {
        aceinna.ProvisionACEINNAInclinometer();
        motionController.PopMessage("FLASHED SENSE EEPROM");
        delay(500);
    }

    // Check if the user wanted to zero the inclinometer
    if (digitalRead(PIN_CAST(Constants::Pins::BUTTON::ZERO))) {
        storageManager.getMap()->zeroFrame1 = inclinometer1.zero();
        storageManager.writeMap();
        motionController.PopMessage("RESET LEVEL SENSOR");
        delay(500);
    }

    // avoid thrashing the sensor too much
    delay(10);
}

void indicator_step(Motion::MotionStateMachine::STATE state)
{
    // Fault indicator
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::FAULT_ACTIVE),
                 Fault::Handler::instance()->hasFault());

    // Ready indicator
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::READY),
                 state == Motion::MotionStateMachine::STATE_MOVING);

    // Fault clearable indicator
    digitalWrite(PIN_CAST(Constants::Pins::INDICATOR::FAULT_CLEARABLE),
                 state == Motion::MotionStateMachine::STATE_FAULTED &&
                     !Fault::Handler::instance()->hasFault());
}
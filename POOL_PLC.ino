#include "ACEINNAInclinometer.h"
#include "FaultHandling.h"
#include "InclinometerModel.h"
#include "InclinometerModule.h"
#include "PersistentStorage.h"
#include "MotionController.h"

#include <stlport.h>

#include <Eigen30.h>

#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>
#include <Controllino.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <SPI.h>

#define STATUS_FLASH_PIN CONTROLLINO_D7
#define ZERO_BUTTON      CONTROLLINO_A0
#define RAISE_BUTTON     CONTROLLINO_A1
#define LOWER_BUTTON     CONTROLLINO_A2

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
    pinMode(STATUS_FLASH_PIN, OUTPUT);
    pinMode(ZERO_BUTTON, INPUT);
    pinMode(RAISE_BUTTON, INPUT);
    pinMode(LOWER_BUTTON, INPUT);
    digitalWrite(STATUS_FLASH_PIN, HIGH);

    SPI.begin();
    Serial.begin(9600);
    Serial.println("Starting up");
    if (!inclinometer1.begin()) {
        faultHandler->setFaultCode(Fault::INCLINOMETER_INIT);
    }
    if (!storageManager.begin()) {
        faultHandler->setFaultCode(Fault::FRAM_INIT);
    }
    storageManager.readMap();
    inclinometer1.importZero(storageManager.getMap()->zeroFrame1);
    motionController.Initialize();
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

    // This just runs the periodic error flasher code
    faultHandler->faultFlasherPeriodic(STATUS_FLASH_PIN);

    // Step the motion controller
    motionController.Step();

    // Check if the user wanted to zero the accelerometers
    if (!digitalRead(ZERO_BUTTON) && !(raising || lowering)) {
        storageManager.getMap()->zeroFrame1 = inclinometer1.zero();
        storageManager.writeMap();
        digitalWrite(STATUS_FLASH_PIN, LOW);
        delay(100);
        digitalWrite(STATUS_FLASH_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_FLASH_PIN, LOW);
        delay(100);
        digitalWrite(STATUS_FLASH_PIN, HIGH);
        delay(100);
    }
    // avoid thrashing the sensor too much
    delay(10);
}

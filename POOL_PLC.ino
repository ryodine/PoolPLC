#include "ACEINNAInclinometer.h"
#include "ADXL355Inclinometer.h"
#include "FaultHandling.h"
#include "HighestCornerAlgorithm.h"
#include "InclinometerModel.h"
#include "InclinometerModule.h"
#include "PersistentStorage.h"

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

#define OUT_TR CONTROLLINO_D16
#define OUT_TL CONTROLLINO_D17
#define OUT_BL CONTROLLINO_D18
#define OUT_BR CONTROLLINO_D19

Fault::Handler *faultHandler;
PersistentStorage::Manager storageManager;

Inclinometer::ACEINNAInclinometer aceinna(Serial2);
Inclinometer::Module inclinometer1(&aceinna, 0.0);

HighestCornerAlgo cornerAlgo(2.5 / 180.0 * PI, 5.0 / 180.0 * PI);

// Inclinometer connection check constants and counters
const int maximumInclinometerRetries = 100;
int noInclinometerDataCounter = 0;

void setup()
{
    faultHandler = Fault::Handler::instance();
    // put your setup code here, to run once:
    pinMode(STATUS_FLASH_PIN, OUTPUT);
    pinMode(OUT_TR, OUTPUT);
    pinMode(OUT_BR, OUTPUT);
    pinMode(OUT_TL, OUTPUT);
    pinMode(OUT_BL, OUTPUT);

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
    Serial.println("Initialized");
    delay(200);
}

void loop()
{
    
    // Determine if raising or lowering
    bool raising = digitalRead(RAISE_BUTTON);
    bool lowering = digitalRead(LOWER_BUTTON);
    raising = raising && !lowering;
    lowering = lowering && !raising;

    // This just runs the periodic error flasher code
    faultHandler->faultFlasherPeriodic(STATUS_FLASH_PIN);

    if ((raising || lowering)) {
        // Check if there is data ready on the inclinometer
        if (inclinometer1.hasData()) {

            // If the inclinometer has data ready, then we can safely unlatch &
            // reset the no data ready fault
            faultHandler->unlatchFaultCode(Fault::INCLINOMETER_NOT_READY);
            noInclinometerDataCounter = 0;

            // Calculate the angles from the data using the mathematical model
            Eigen::Vector2d angles = inclinometer1.getData();
            Serial.print(angles[0] * 180.0 / PI);
            Serial.print("\t");
            Serial.print(angles[1] * 180.0 / PI);
            Serial.print("\n");

            // Pass the calculated angles into the highest corner finding
            // algorithm
            cornerAlgo.update(angles[0], angles[1]);

            // Control the solenoids, if no faults and in raise or lower mode
            if (!faultHandler->hasFault()) {
                digitalWrite(OUT_TR, cornerAlgo.getCorner(0, lowering));
                digitalWrite(OUT_TL, cornerAlgo.getCorner(1, lowering));
                digitalWrite(OUT_BL, cornerAlgo.getCorner(2, lowering));
                digitalWrite(OUT_BR, cornerAlgo.getCorner(3, lowering));
            }
        }
        else {
            // If there isn't data ready for noInclinometerDataCounter tries,
            // then register an temporary fault.
            if (noInclinometerDataCounter++ > maximumInclinometerRetries) {
                faultHandler->setFaultCode(Fault::INCLINOMETER_NOT_READY);
            }
        }
    }

    // if there are faults, do not control the solenoids.
    if (faultHandler->hasFault() || !(raising || lowering)) {
        digitalWrite(OUT_TR, LOW);
        digitalWrite(OUT_TL, LOW);
        digitalWrite(OUT_BL, LOW);
        digitalWrite(OUT_BR, LOW);
    }

    // Check if the user wanted to zero the accelerometers
    if (!digitalRead(ZERO_BUTTON) && !(raising || lowering)) {
        if (inclinometer1.hasData()) {
            storageManager.getMap()->zeroFrame1 = inclinometer1.zero();
            storageManager.writeMap();
            faultHandler->unlatchFaultCode(Fault::INCLINOMETER_NOT_READY);
            digitalWrite(STATUS_FLASH_PIN, LOW);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, LOW);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, HIGH);
        }
        else {
            faultHandler->setFaultCode(Fault::INCLINOMETER_NOT_READY);
        }
        delay(100);
    }

    // avoid thrashing the sensor too much
    delay(50);
}

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

Inclinometer::ADXL355Inclinometer accel1(CONTROLLINO_D4, CONTROLLINO_D0);
Inclinometer::ADXL355Inclinometer accel2(CONTROLLINO_D2, CONTROLLINO_D0);
Inclinometer::Module inclinometer1(&accel1, 0.0);
Inclinometer::Module inclinometer2(&accel2, 0.0);

HighestCornerAlgo cornerAlgo(2.5 / 180.0 * PI, 5.0 / 180.0 * PI);

// Accelerometer implausibility check constants and counters
const int max_accel_retries = 100;
const double accel_max_gravity_deviation_fraction = 0.5;
int accelBusyCounter = 0;

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
    if (!inclinometer1.begin()) {
        faultHandler->setFaultCode(Fault::ACCEL_INIT);
    }
    if (!inclinometer2.begin()) {
        faultHandler->setFaultCode(Fault::ACCEL_INIT2);
    }
    if (!storageManager.begin()) {
        faultHandler->setFaultCode(Fault::FRAM_INIT);
    }
    storageManager.readMap();
    inclinometer1.importZero(storageManager.getMap()->zeroFrame1);
    inclinometer2.importZero(storageManager.getMap()->zeroFrame2);

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
        // Check if there is data ready on the accelerometer
        if (inclinometer1.hasData() && inclinometer2.hasData()) {

            // If the accelerometer has data ready, then we can safely unlatch &
            // reset the no data ready fault
            faultHandler->unlatchFaultCode(Fault::ACCEL_NOT_READY);
            accelBusyCounter = 0;

            // Calculate the angles from the data using the mathematical model
            Eigen::Vector2d angles = inclinometer1.getData();
            Serial.print(angles[0] * 180.0 / PI);
            Serial.print("\t");
            Serial.print(angles[1] * 180.0 / PI);
            Serial.print("\t");

            Eigen::Vector2d angles2 = inclinometer2.getData();
            Serial.print(angles2[0] * 180.0 / PI);
            Serial.print("\t");
            Serial.print(angles2[1] * 180.0 / PI);
            Serial.println();

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
            // If there isn't data ready for max_accel_retries tries, then
            // register an temporary fault.
            if (accelBusyCounter++ > max_accel_retries) {
                faultHandler->setFaultCode(Fault::ACCEL_NOT_READY);
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
        if (inclinometer1.hasData() && inclinometer2.hasData()) {
            storageManager.getMap()->zeroFrame1 = inclinometer1.zero();
            storageManager.getMap()->zeroFrame2 = inclinometer2.zero();
            storageManager.writeMap();
            faultHandler->unlatchFaultCode(Fault::ACCEL_NOT_READY);
            digitalWrite(STATUS_FLASH_PIN, LOW);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, LOW);
            delay(100);
            digitalWrite(STATUS_FLASH_PIN, HIGH);
        }
        else {
            faultHandler->setFaultCode(Fault::ACCEL_NOT_READY);
        }
        delay(100);
    }

    // avoid thrashing the sensor too much
    delay(10);
}

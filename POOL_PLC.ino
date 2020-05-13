#include <SPI.h>
#include <Controllino.h>
#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "AccelerometerFiltering.h"
#include "AccelerometerModel.h"
#include "ADXL355.h"
#include "FaultHandling.h"
#include "HighestCornerAlgorithm.h"

#define STATUS_FLASH_PIN CONTROLLINO_D0
#define ZERO_BUTTON CONTROLLINO_A0

#define OUT_TR CONTROLLINO_D16
#define OUT_TL CONTROLLINO_D17
#define OUT_BL CONTROLLINO_D18
#define OUT_BR CONTROLLINO_D19

Fault::Handler* faultHandler;
ADXL355 accel(CONTROLLINO_D4);
AccelerometerFilterMovingAverage ewmaFilter(0, 0.05);
AccelerometerModel accelModel;
HighestCornerAlgo cornerAlgo(2.5 / 180.0 * PI, 5.0 / 180.0 * PI);

// Accelerometer implausibility check constants and counters
const int max_accel_retries = 10;
const double accel_max_gravity_deviation_fraction = 0.5;
int accelBusyCounter = 0;

void setup() {
  faultHandler = Fault::Handler::instance();
  // put your setup code here, to run once:
  pinMode(STATUS_FLASH_PIN, OUTPUT);
  pinMode(OUT_TR, OUTPUT);
  pinMode(OUT_BR, OUTPUT);
  pinMode(OUT_TL, OUTPUT);
  pinMode(OUT_BL, OUTPUT);
  
  pinMode(ZERO_BUTTON, INPUT);
  digitalWrite(STATUS_FLASH_PIN, HIGH);
  
  SPI.begin();
  Serial.begin(9600);
  // ADXL355_FILTER_LPF_LOW_FREQ
  if (!accel.begin(ADXL355_RANGE_2G, ADXL355_FILTER_LPF_16HZ_ODR)) {
    faultHandler->setFaultCode(Fault::ACCEL_INIT);
  }

  delay(200);

  //Allows a preconfigured offset (useful for YAW)
  //All rotations are in Roll-Pitch-Yaw (Left to right).
  accelModel.setBaseFrameAnglesRadians(Eigen::Vector3d(0,0,0));
  
}

void loop() {

  // This just runs the periodic error flasher code
  faultHandler->faultFlasherPeriodic(STATUS_FLASH_PIN);
  
  ADXL355Measurement measure;

  // Check if there is data ready on the accelerometer
  if (accel.dataReady()) {

    // If the accelerometer has data ready, then we can safely unlatch & reset the no data ready fault
    faultHandler->unlatchFaultCode(Fault::ACCEL_NOT_READY);
    accelBusyCounter = 0;

    // Sample the accelerometer
    accel.takeSample();
    measure = accel.getSample();

    // Magnitude plausibility check
    magnitudePlausibilityCheck(measure);

    // Filter the data with an Expoentially Weighted Moving Average
    ewmaFilter.addData(measure);
    measure = ewmaFilter.getAverage();

    // Calculate the angles from the data using the mathematical model
    Eigen::Vector2d angles = accelModel.calculate(Eigen::Vector3d(measure.x, measure.y, measure.z));
    Serial.print(angles[0] * 180.0 / PI);
    Serial.print("\t");
    Serial.print(angles[1] * 180.0 / PI);
    Serial.println();

    // Pass the calculated angles into the highest corner finding algorithm
    cornerAlgo.update(angles[0], angles[1]);

    // Control the solenoids, if no faults
    if (!faultHandler->hasFault()) {
      digitalWrite(OUT_TR, cornerAlgo.getCorner(0));
      digitalWrite(OUT_TL, cornerAlgo.getCorner(1));
      digitalWrite(OUT_BL, cornerAlgo.getCorner(2));
      digitalWrite(OUT_BR, cornerAlgo.getCorner(3));
    }
    
  } else {
    // If there isn't data ready for max_accel_retries tries, then register an temporary fault.
    if (accelBusyCounter++ > max_accel_retries) {
      faultHandler->setFaultCode(Fault::ACCEL_NOT_READY);
    }
  }

  // if there are faults, do not control the solenoids.
  if (faultHandler->hasFault()) {
    digitalWrite(OUT_TR, LOW);
    digitalWrite(OUT_TL, LOW);
    digitalWrite(OUT_BL, LOW);
    digitalWrite(OUT_BR, LOW);
  }

  // Check if the user wanted to zero the accelerometers
  if (!digitalRead(ZERO_BUTTON)) {
    accelModel.setMeasurementAsZero(Eigen::Vector3d(measure.x, measure.y, measure.z));
  }

  // avoid thrashing the sensor too much
  delay(10);
}

/**
 * Checks that the accelerometer is outputing values that are plausible,
 * by checking that the magnitude of the force vector is close to gravity.
 * Sets the fault ACCEL_IMPLAUSIBLE_READING if there is an error
 * 
 * @param measure the measurement from the accelerometer
 */
void magnitudePlausibilityCheck(ADXL355Measurement measure)
{
  double magnitude = sqrt(pow(measure.x/16.0, 2) + pow(measure.y/16.0, 2) + pow(measure.z/16.0, 2));
  if (magnitude > 1.0+accel_max_gravity_deviation_fraction || magnitude < 1.0-accel_max_gravity_deviation_fraction) {
    faultHandler->setFaultCode(Fault::ACCEL_IMPLAUSIBLE_READING);
  } else {
    faultHandler->unlatchFaultCode(Fault::ACCEL_IMPLAUSIBLE_READING);
  }
}

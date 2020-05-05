#include <SPI.h>
#include <Controllino.h>
#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "AccelerometerFiltering.h"
#include "AccelerometerModel.h"
#include "ADXL355.h"
#include "HighestCornerAlgorithm.h"

#define STATUS_FLASH_PIN CONTROLLINO_D0
#define ZERO_BUTTON CONTROLLINO_A0

#define OUT_TR CONTROLLINO_D16
#define OUT_TL CONTROLLINO_D17
#define OUT_BL CONTROLLINO_D18
#define OUT_BR CONTROLLINO_D19

ADXL355 accel(CONTROLLINO_D4);
AccelerometerFilterMovingAverage ewmaFilter(0, 0.15);
AccelerometerModel accelModel;
HighestCornerAlgo cornerAlgo(2.5 / 180.0 * PI, 5.0 / 180.0 * PI);

void setup() {
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
  while (!accel.begin(ADXL355_RANGE_2G, ADXL355_FILTER_LPF_16HZ_ODR)) {
    digitalWrite(STATUS_FLASH_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_FLASH_PIN, HIGH);
    delay(100);
    Serial.println("ADXL355 Failed to initialize");
  }
  Serial.println("ADXL355 Initialized");

  delay(200);

  //Allows a preconfigured offset (useful for YAW)
  //All rotations are in Roll-Pitch-Yaw (Left to right).
  accelModel.setBaseFrameAnglesRadians(Eigen::Vector3d(0,0,0));
}

void loop() {
  ADXL355Measurement measure;
  if (accel.dataReady()) {
    accel.takeSample();
  
    measure = accel.getSample();
    ewmaFilter.addData(measure);
    measure = ewmaFilter.getAverage();
  
    Eigen::Vector2d angles = accelModel.calculate(Eigen::Vector3d(measure.x, measure.y, measure.z));
    Serial.print(angles[0] * 180.0 / PI);
    Serial.print("\t");
    Serial.print(angles[1] * 180.0 / PI);
    Serial.println();
  
    cornerAlgo.update(angles[0], angles[1]);
  
    digitalWrite(OUT_TR, cornerAlgo.getCorner(0));
    digitalWrite(OUT_TL, cornerAlgo.getCorner(1));
    digitalWrite(OUT_BL, cornerAlgo.getCorner(2));
    digitalWrite(OUT_BR, cornerAlgo.getCorner(3));
  }

  if (!digitalRead(ZERO_BUTTON)) {
    accelModel.setMeasurementAsZero(Eigen::Vector3d(measure.x, measure.y, measure.z));
  }

  // avoid thrashing the sensor too much
  delay(10);
}

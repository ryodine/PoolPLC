#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <SPI.h>
#include <Controllino.h>
#include "ADXL355.h"
#include "AccelerometerFiltering.h"
#include "AccelerometerModel.h"

#define STATUS_FLASH_PIN CONTROLLINO_D0
#define ZERO_BUTTON CONTROLLINO_A0

ADXL355 accel(CONTROLLINO_D4);
AccelerometerFilterMovingAverage ewmaFilter(0, 0.2);
AccelerometerModel accelModel;

void setup() {
  // put your setup code here, to run once:
  pinMode(STATUS_FLASH_PIN, OUTPUT);
  pinMode(ZERO_BUTTON, INPUT);
  digitalWrite(STATUS_FLASH_PIN, HIGH);
  
  SPI.begin();
  Serial.begin(9600);
  while (!accel.begin(ADXL355_RANGE_2G, ADXL355_FILTER_LPF_LOW_FREQ)) {
    digitalWrite(STATUS_FLASH_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_FLASH_PIN, HIGH);
    delay(100);
    Serial.println("ADXL355 Failed to initialize");
  }
  Serial.println("ADXL355 Initialized");

  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  accel.takeSample();

  ADXL355Measurement measure = accel.getSample();
  ewmaFilter.addData(measure);
  measure = ewmaFilter.getAverage();

  Eigen::Vector2d angles = accelModel.calculate(Eigen::Vector3d(measure.x, measure.y, measure.z));
  Serial.print(angles[0] * 180.0 / PI);
  Serial.print("\t");
  Serial.print(angles[1] * 180.0 / PI);
  Serial.print("\t");
  Serial.print(digitalRead(ZERO_BUTTON));
  Serial.println();
  /*Vect3D vect = UnitVectorizeMeasurement::vectorize(measure);

  double pitch = atan(vect.components[1]/vect.components[2]);
  double roll = atan((-vect.components[0])/sqrt(pow(vect.components[1], 2)+pow(vect.components[2], 2)));*/

  /*Serial.print(pitch * 180.0 / PI);
  Serial.print("\t");
  Serial.print(roll  * 180.0 / PI);
  Serial.print("\t");
  Serial.println();*/

  /*Eigen::Matrix3d m_0;
  m_0 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(PI/2.0, Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());*/
  /*for (int i = 0; i < 3; i++) {
    Serial.print(m_0(i,0));
    Serial.print("\t");
    Serial.print(m_0(i,1));
    Serial.print("\t");
    Serial.print(m_0(i,2));
    Serial.print("\n");
  }
  Serial.println();*/

  /*Eigen::Matrix3d m_1;
  m_1 = Eigen::AngleAxisd(PI/4, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  
  Eigen::Vector3d ea = (m_1*m).eulerAngles(0, 1, 2);
  Serial.print(ea[0] * 180.0 / PI);
  Serial.print("\t");
  Serial.print(ea[1] * 180.0 / PI);
  Serial.print("\t");

  Eigen::Vector3d ea2 = (m).eulerAngles(0, 1, 2);
  Serial.print(ea2[0] * 180.0 / PI);
  Serial.print("\t");
  Serial.print(ea2[1] * 180.0 / PI);
  Serial.print("\t");
  Serial.println("");*/
  
  /*Serial.print(vect.components[0]/16.0);
  Serial.print("\t");
  Serial.print(vect.components[1]/16.0);
  Serial.print("\t");
  Serial.print(vect.components[2]/16.0);
  Serial.println("");*/
}

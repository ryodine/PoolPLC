#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <SPI.h>
#include <Controllino.h>
#include "ADXL355.h"
#include "AccelerometerFiltering.h"

ADXL355 accel(CONTROLLINO_D4);
AccelerometerFilterMovingAverage ewmaFilter(0, 0.1);

void setup() {
  // put your setup code here, to run once:

  SPI.begin();
  Serial.begin(9600);
  accel.begin(ADXL355_RANGE_2G, ADXL355_FILTER_LPF_LOW_FREQ);

  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  accel.takeSample();

  ADXL355Measurement measure = accel.getSample();
  ewmaFilter.addData(measure);
  measure = ewmaFilter.getAverage();
  Vect3D vect = UnitVectorizeMeasurement::vectorize(measure);

  double pitch = atan(vect.components[1]/vect.components[2]);
  double roll = atan((-vect.components[0])/sqrt(pow(vect.components[1], 2)+pow(vect.components[2], 2)));

  /*Serial.print(pitch * 180.0 / PI);
  Serial.print("\t");
  Serial.print(roll  * 180.0 / PI);
  Serial.print("\t");
  Serial.println();*/

  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ());
  /*for (int i = 0; i < 3; i++) {
    Serial.print(m(i,0));
    Serial.print("\t");
    Serial.print(m(i,1));
    Serial.print("\t");
    Serial.print(m(i,2));
    Serial.print("\n");
  }*/
  Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);
  Serial.print(ea[0] * 180.0 / PI);
  Serial.print("\t");
  Serial.print(ea[1] * 180.0 / PI);
  Serial.print("\t");
  Serial.println("");
  
  /*Serial.print(vect.components[0]/16.0);
  Serial.print("\t");
  Serial.print(vect.components[1]/16.0);
  Serial.print("\t");
  Serial.print(vect.components[2]/16.0);
  Serial.println("");*/
  
}

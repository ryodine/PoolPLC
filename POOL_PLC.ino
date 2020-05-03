#include <SPI.h>
#include <Controllino.h>
#include "ADXL355.h"
#include "MovingAverage.h"

ADXL355 accel(CONTROLLINO_D4);
double alpha = 0.1;
MovingAverage avgs[3] = { MovingAverage(0, alpha),MovingAverage(0, alpha),MovingAverage(0, alpha) };

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
  for (int i = 0; i < 3; i++) {
    avgs[i].addPoint(((double*)&measure)[i]);
  }

  for (int i = 0; i < 3; i++) {
    Serial.print(avgs[i].getAverage());
    Serial.print("\t");
  }
  Serial.println();
  /*Serial.print(measure.x);
  Serial.print("\t");
  Serial.print(measure.y);
  Serial.print("\t");
  Serial.print(measure.z);
  Serial.println("");*/
  
}

#include "HighestCornerAlgorithm.h"
#include <cmath>

void HighestCornerAlgo::update(double roll, double pitch)
{
  if (roll > upperbound) {
    if (pitch > upperbound) {
      resetAll();
      corners[2] = true;
    } else if (pitch < -upperbound) {
      resetAll();
      corners[1] = true;
    } else {
      resetAll();
      corners[1] = true;
      corners[2] = true;
    }
  } else if (roll < -upperbound) {
    if (pitch > upperbound) {
      resetAll();
      corners[3] = true;
    } else if (pitch < -upperbound) {
      resetAll();
      corners[0] = true;
    } else {
      resetAll();
      corners[0] = true;
      corners[3] = true;
    }
  } else {
    if (pitch > upperbound) {
      resetAll();
      corners[2] = true;
      corners[3] = true;
    } else if (pitch < -upperbound) {
      resetAll();
      corners[0] = true;
      corners[1] = true;
    } else {
      // OK!!
    }
  }

  if ((roll < 0 && roll > -lowerbound || roll > 0 && roll < lowerbound)
      && (pitch < 0 && pitch > -lowerbound || pitch > 0 && pitch < lowerbound)) {
    resetAll();
  }
}

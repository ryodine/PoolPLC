#ifndef PERSISTENT_STORAGE_H
#define PERSISTENT_STORAGE_H

#include "Adafruit_FRAM_I2C.h"
#include "InclinometerModel.h"

#include <Wire.h>

namespace PersistentStorage {
typedef struct {
    Inclinometer::ModelZeropoint zeroFrame1;
    Inclinometer::ModelZeropoint zeroFrame2;
} Map;

class Manager {
  public:
    bool begin() { return fram.begin(); };
    void readMap();
    void writeMap();
    Map *getMap() { return &storageMap; };

  private:
    Map storageMap;
    Adafruit_FRAM_I2C fram;
};
}; // namespace PersistentStorage

#endif
/**
 * @file PersistentStorage.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief An interface to write persistent data to the Adafruit FRAM I2C Module.
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef PERSISTENT_STORAGE_H
#define PERSISTENT_STORAGE_H

#include "Adafruit_FRAM_I2C.h"
#include "InclinometerModel.h"

#include <Wire.h>

namespace PersistentStorage {

/**
 * @brief PersistentStorage memory map
 */
typedef struct {
    Inclinometer::ModelZeropoint zeroFrame1;
    Inclinometer::ModelZeropoint zeroFrame2;
} Map;

/**
 * @brief PersistentStorage memory manager
 */
class Manager {
  public:
    /**
     * @brief Initializes the memory module
     *
     * @return true if the module successfully initialized
     * @return false if the module did not successfully initialized
     */
    bool begin() { return fram.begin(); };

    /**
     * @brief read the memory from persistent storage into the manager
     */
    void readMap();

    /**
     * @brief write the memory from the manager into persistent storage
     */
    void writeMap();

    /**
     * @brief Gets a pointer to the memory map that can be read or edited
     * between readMap() and writeMap() operations.
     *
     * @return Map*
     */
    Map *getMap() { return &storageMap; };

  private:
    Map storageMap;
    Adafruit_FRAM_I2C fram;
};
}; // namespace PersistentStorage

#endif
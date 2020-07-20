#include "PersistentStorage.h"

void PersistentStorage::Manager::readMap()
{
    uint8_t *mapBuffer = (uint8_t *)&storageMap;
    for (unsigned int i = 0; i < sizeof(Map); i++) {
        mapBuffer[i] = fram.read8(i);
    }
}

void PersistentStorage::Manager::writeMap()
{
    uint8_t *mapBuffer = (uint8_t *)&storageMap;
    for (unsigned int i = 0; i < sizeof(Map); i++) {
        fram.write8(i, mapBuffer[i]);
    }
}
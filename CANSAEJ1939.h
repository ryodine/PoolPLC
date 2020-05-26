/**
 * Implements basic features of the SAEJ1939 Can2.0B Spec
 */

#ifndef CAN_SAEJ1939_Layer_H
#define CAN_SAEJ1939_Layer_H

#include "CANInterface.h"

namespace CAN {
class J1939ID {
  public:
    J1939ID() : canID(0), priority(0), sourceAddress(0), PGN(0){};
    J1939ID(byte priority, byte sourceAddress, unsigned long PGN)
        : canID(((unsigned long)priority << 26) + (PGN << 8) +
                (unsigned long)sourceAddress),
          priority(priority), sourceAddress(sourceAddress), PGN(PGN){};
    J1939ID(unsigned long ID)
        : canID(ID), PGN((ID & (long)0xFFFF00) >> 8),
          priority((ID >> 26) & 0b111), sourceAddress(ID & 0xFF){};
    unsigned long getID() { return canID; };
    byte getPriority() { return priority; };
    byte getSourceAddress() { return sourceAddress; };
    unsigned long getPGN() { return PGN; };

  private:
    unsigned long canID;
    byte priority;
    byte sourceAddress;
    unsigned long PGN;
};

struct J1939Message {
    J1939ID CanID;
    byte data[8];

    J1939Message(byte source, byte dest, unsigned long PGN, byte data1)
    {
        data[0] = dest;
        data[1] = data1;
        CanID = J1939ID(6, source, PGN);
    }

    J1939Message(byte source, byte dest, unsigned long PGN, byte data1,
                 byte data2)
    {
        data[0] = dest;
        data[1] = data1;
        data[2] = data2;
        CanID = J1939ID(6, source, PGN);
    }

    J1939Message(ExtendedCanDataPacket p)
    {
        unsigned long id = p.id;
        memcpy(&data, &p.data, 8);
        CanID = J1939ID(id);
    }
}; // namespace CAN

class J1939Interface : public RawInterface {
  public:
    J1939Interface(HardwareSerial &serialInterface)
        : RawInterface(serialInterface){};
    void write(J1939Message p, byte dest);
    J1939Message read();
    void writeRaw(ExtendedCanDataPacket &p) { return RawInterface::write(p); };
    bool readRaw(ExtendedCanDataPacket &p) { return RawInterface::read(p); };

  private:
    bool j1939PeerToPeer(unsigned long lPGN);
};
} // namespace CAN

#endif
/**
 * @file CANSAEJ1939.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Implements basic features of the SAEJ1939 Can2.0B Spec
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef CAN_SAEJ1939_Layer_H
#define CAN_SAEJ1939_Layer_H

#include "CANInterface.h"

namespace CAN {
/**
 * @brief The J1939ID Class builds and parses 29-bit can IDs.
 *
 * A SAEJ1939 CAN ID uses the extended identifiers in CAN2.0
 * This is formatted as follows:
 * MSB: 3 Bits - Priority
 *     18 Bits - PGN (Parameter Group Number)
 * LSB: 8 Bits - Source Address
 */
class J1939ID {
  public:
    /**
     * @brief Construct a new empty J1939ID object
     */
    J1939ID() : canID(0), priority(0), sourceAddress(0), PGN(0){};

    /**
     * @brief Construct a new J1939ID object from Priority, PGN, Source Address
     *
     * @param priority the CAN message priority
     * @param sourceAddress the source address of the packet
     * @param PGN the Parameter Group Number of the message
     */
    J1939ID(byte priority, byte sourceAddress, unsigned long PGN)
        : canID(((unsigned long)priority << 26) + (PGN << 8) +
                (unsigned long)sourceAddress),
          priority(priority), sourceAddress(sourceAddress), PGN(PGN){};

    /**
     * @brief Construct a new J1939ID object by parsing an existing ID
     *
     * @param ID the existing 29-bit can ID in a 32 bit container
     */
    J1939ID(unsigned long ID)
        : canID(ID), PGN((ID & (long)0xFFFF00) >> 8),
          priority((ID >> 26) & 0b111), sourceAddress(ID & 0xFF){};

    /**
     * @brief Get the CAN ID represented by this object
     *
     * @return unsigned long the 29-but CAN ID represented by this object, with
     * zero padding in front.
     */
    unsigned long getID() { return canID; };

    /**
     * @brief Get the Priority
     *
     * @return byte priority byte (only valid to 3 bits)
     */
    byte getPriority() { return priority; };

    /**
     * @brief Get the Source Address
     *
     * @return byte the source address
     */
    byte getSourceAddress() { return sourceAddress; };

    /**
     * @brief Get the PGN
     *
     * @return unsigned long the Parameter Group Number of this ID
     */
    unsigned long getPGN() { return PGN; };

  private:
    unsigned long canID;
    byte priority;
    byte sourceAddress;
    unsigned long PGN;
};

/**
 * @brief This structure represents a SAEJ1939 Message comprised of a J1939ID
 * Object and 8 bytes of data
 *
 */
struct J1939Message {

    //! The CanID of this packet
    J1939ID CanID;

    //! 8 bytes of data, zero-filled initially
    byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    /**
     * @brief Construct a new J1939Message object representing a command message
     * with one byte of data in the payload
     *
     * @param source source CAN address of the message
     * @param dest destination CAN address of the message
     * @param PGN parameter group number of the message
     * @param data1 the one byte of data (leftmost byte) to send, filling the
     * rest of the bytes with zeros
     */
    J1939Message(byte source, byte dest, unsigned long PGN, byte data1)
    {
        data[0] = dest;
        data[1] = data1;
        CanID = J1939ID(6, source, PGN);
    }

    /**
     * @brief Construct a new J1939Message object representing a command message
     * with two bytes of data in the payload
     *
     * @param source source CAN address of the message
     * @param dest destination CAN address of the message
     * @param PGN parameter group number of the message
     * @param data1 the first byte of data (leftmost byte) to send, filling the
     * rest of the bytes with zeros
     * @param data2 the second byte of data (second-to-leftmost byte) to send,
     * filling the rest of the bytes with zeros
     */
    J1939Message(byte source, byte dest, unsigned long PGN, byte data1,
                 byte data2)
    {
        data[0] = dest;
        data[1] = data1;
        data[2] = data2;
        CanID = J1939ID(6, source, PGN);
    }

    /**
     * @brief Construct a new J1939Message object by parsing a CAN2.0B extended
     * packet
     *
     * @param p the packet to parse
     */
    J1939Message(ExtendedCanDataPacket p)
    {
        unsigned long id = p.id;
        memcpy(&data, &p.data, 8);
        CanID = J1939ID(id);
    }
}; // namespace CAN

/**
 * @brief This interface allows writing to the Serial Can Module using the
 * SAEJ1939 CAN 2.0 Protocol stack
 *
 */
class J1939Interface : public RawInterface {
  public:
    /**
     * @brief Construct a new J1939Interface object.
     *
     * @param serialInterface the HardwareSerial device to which the CAN Bus
     * module is connected
     */
    J1939Interface(HardwareSerial &serialInterface)
        : RawInterface(serialInterface){};

    /**
     * @brief Write a J1939 Message to the CAN Bus.
     *
     * @param p the J1939 Message to send
     * @param dest the address to send the packet to. The destination address is
     * used to determine if the packet is peer-to-peer.
     */
    void write(J1939Message p, byte dest);

    /**
     * @brief Read a J1939 Message from the CAN Bus
     *
     * @return J1939Message that was read. If no message was read, the message
     * will be initialized empty (i.e. CAN ID is 0, data is 0)
     */
    J1939Message read();

    /**
     * @see CAN::RawInterface::write(ExtendedCanDataPacket p)
     */
    void writeRaw(ExtendedCanDataPacket &p) { return RawInterface::write(p); };

    /**
     * @see CAN::RawInterface::read(ExtendedCanDataPacket p)
     */
    bool readRaw(ExtendedCanDataPacket &p) { return RawInterface::read(p); };

  private:
    bool j1939PeerToPeer(unsigned long lPGN);
};
} // namespace CAN

#endif
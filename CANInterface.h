/**
 * @file CANInterface.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief This class interfaces with the Serial UART to CAN Bus controller from
 * Longan Labs, based on specifications from
 * https://docs.longan-labs.cc/1030001/
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef LONGAN_CAN_UART_ADAPTER_INTERFACE_H
#define LONGAN_CAN_UART_ADAPTER_INTERFACE_H

#include <Arduino.h>

namespace CAN {
/**
 * @brief Baudrates that the Serial CAN Module will accept.
 */
enum SerialBaudrate {
    // b/s
    baud_START = 0,
    baud_9600 = baud_START,
    baud_19200,
    baud_38400,
    baud_57600,
    baud_115200,
    baud_END
};

/**
 * @brief Baudrated that the CAN bus can run at
 */
enum CANBusBaudrate {
    // kb/s
    kbps_START = 1,
    kbps_5 = kbps_START,
    kbps_10,
    kbps_20,
    kbps_25,
    kbps_31_2,
    kbps_33,
    kbps_40,
    kbps_50,
    kbps_80,
    kbps_83_3,
    kbps_95,
    kbps_100,
    kbps_125,
    kbps_200,
    kbps_250,
    kbps_500,
    kbps_666,
    kbps_1000,
    kbps_END
};

/**
 * @brief This structure represents an extended can 2.0 packet with no
 * additional formatting
 */
typedef struct {
    unsigned long id;
    byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} ExtendedCanDataPacket;

/**
 * @brief This class interfaces with the Serial CAN Module without any higher
 * level protocols.
 */
class RawInterface {
  public:
    /**
     * @brief Construct a new Raw Interface object
     *
     * @param serialInterface the HardwareSerial device to which the CAN Bus
     * module is connected
     */
    RawInterface(HardwareSerial &serialInterface);

    /**
     * @brief initializes the module
     *
     * @param serial Serial UART speed to run the module at (default 115200)
     * @param can CAN Bus baudrate to run at
     */
    void begin(SerialBaudrate serial = SerialBaudrate::baud_115200,
               CANBusBaudrate can = CANBusBaudrate::kbps_250);

    /**
     * @brief Writes a packet to the controller
     *
     * @param p the data packet to provide to the CAN module
     * @param extBit true if using 29-bit IDs, false if using 11-bit IDs
     * (default false)
     * @param rtrBit true if the packet is a RTR packet, meaning a request for
     * data (Default false)
     */
    void write(ExtendedCanDataPacket p, bool extBit = true,
               bool rtrBit = false);

    /**
     * @brief Reads a packet from the controller, if one is available
     *
     * NOTE: If the packet ID makes no sense, the read buffer might be offset.
     * In this case, please call the flushBuffer() method
     *
     * @param p reference to the packet object to overwrite the contents of
     * @return true if the packet was read
     * @return false if no packet was read
     */
    bool read(ExtendedCanDataPacket &p);

    /**
     * @brief Checks if a packet can be read
     *
     * Does not check if a packet is valid, just if there are enough bytes in
     * the buffer to read a packet
     *
     * @return true if a packet can be read
     * @return false if a packet is not yet available
     */
    bool hasPacket();

    /**
     * @brief Clears out the receive buffer completely
     */
    void flushBuffer();

  private:
    HardwareSerial &serialInterface;
    void resetBaudTo(SerialBaudrate s);
    unsigned long serialBaudToBaud(SerialBaudrate s);
};
} // namespace CAN

#endif
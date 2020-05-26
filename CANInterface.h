/**
 * This class interfaces with the Serial UART to CAN Bus controller.
 * https://docs.longan-labs.cc/1030001/
 */

#ifndef LONGAN_CAN_UART_ADAPTER_INTERFACE_H
#define LONGAN_CAN_UART_ADAPTER_INTERFACE_H

#include <Arduino.h>

namespace CAN {
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
typedef struct {
    unsigned long id;
    byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} ExtendedCanDataPacket;

class RawInterface {
  public:
    RawInterface(HardwareSerial& serialInterface);
    void begin(SerialBaudrate serial = SerialBaudrate::baud_115200,
               CANBusBaudrate can = CANBusBaudrate::kbps_250);
    void write(ExtendedCanDataPacket p, bool extBit = true,
               bool rtrBit = false);
    bool read(ExtendedCanDataPacket &p);
    bool hasPacket();
    void flushBuffer();

  private:
    HardwareSerial& serialInterface;
    void resetBaudTo(SerialBaudrate s);
    unsigned long serialBaudToBaud(SerialBaudrate s);
};
} // namespace CAN

#endif
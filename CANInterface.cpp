#include "CANInterface.h"

CAN::RawInterface::RawInterface(HardwareSerial &serialInterface)
    : serialInterface(serialInterface)
{
}

unsigned long CAN::RawInterface::serialBaudToBaud(CAN::SerialBaudrate s)
{
    switch (s) {
    case SerialBaudrate::baud_9600:
        return 9600;
    case SerialBaudrate::baud_19200:
        return 19200;
    case SerialBaudrate::baud_38400:
        return 38400;
    case SerialBaudrate::baud_57600:
        return 57600;
    case SerialBaudrate::baud_115200:
        return 115200;
    }
}

void CAN::RawInterface::resetBaudTo(CAN::SerialBaudrate s)
{
    for (int baud = baud_START; baud < baud_END; ++baud) {
        serialInterface.begin(serialBaudToBaud(baud));
        serialInterface.print("+++");
        delay(10);
        char atCommand[7];
        sprintf(atCommand, "AT+S=%d", (unsigned int)s);
        atCommand[6] = '\0';
        serialInterface.println(atCommand);
        delay(100);
        Serial.println("did one");
        flushBuffer();
    }
    serialInterface.begin(serialBaudToBaud(s));
    serialInterface.print("+++");
    /*char atCommand[7];
    sprintf(atCommand, "AT+S=%d", (unsigned int)s);
    atCommand[6] = '\0';
    serialInterface.begin(9600);
    serialInterface.print("+++");
    serialInterface.println(atCommand);
    delay(100);
    serialInterface.begin(19200);
    serialInterface.print("+++");
    serialInterface.println(atCommand);
    delay(100);

    serialInterface.begin(38400);
    serialInterface.print("+++");
    serialInterface.println(atCommand);
    delay(100);

    serialInterface.begin(57600);
    serialInterface.print("+++");
    serialInterface.println(atCommand);
    delay(100);

    serialInterface.begin(115200);
    serialInterface.print("+++");
    serialInterface.println(atCommand);
    delay(100);

    serialInterface.begin(serialBaudToBaud(s));
    serialInterface.print("+++");*/
    flushBuffer();
}

void CAN::RawInterface::begin(
    CAN::SerialBaudrate serial = CAN::SerialBaudrate::baud_115200,
    CAN::CANBusBaudrate can = CAN::CANBusBaudrate::kbps_250)
{
    // Set the baudrate of the serial bus
    resetBaudTo(serial);

    // Set the baudrate of the can bus
    char atCommand[8];
    sprintf(atCommand, "AT+C=%d", (unsigned int)can);
    serialInterface.println(atCommand);

    // Set a read mask
    serialInterface.println("AT+M=[0][1][1FFFFFFF]");
    delay(100);

    // Enter data mode
    serialInterface.println("AT+Q");
    delay(100);

    // Clear the read buffer
    flushBuffer();
}

void CAN::RawInterface::flushBuffer()
{
    while (serialInterface.available())
        serialInterface.read();
}

void CAN::RawInterface::write(ExtendedCanDataPacket p, bool extBit = true,
                              bool rtrBit = false)
{
    unsigned char dta[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    unsigned long id = p.id;
    dta[0] = id >> 24;        // id3
    dta[1] = id >> 16 & 0xff; // id2
    dta[2] = id >> 8 & 0xff;  // id1
    dta[3] = id & 0xff;       // id0

    dta[4] = (extBit == true) ? 1 : 0;
    dta[5] = (rtrBit == true) ? 1 : 0;

    for (int i = 0; i < 8; i++) {
        dta[6 + i] = p.data[i];
    }

    for (int i = 0; i < 14; i++) {
        serialInterface.write(dta[i]);
    }
}

bool CAN::RawInterface::read(ExtendedCanDataPacket &p)
{
    if (hasPacket()) {
        byte buf[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 12; i++) {
            buf[i] = serialInterface.read();
        }
        unsigned long id = 0;
        byte *data = buf + 4;
        for (int i = 0; i < 4; i++) {
            id <<= 8;
            id += buf[i];
        }
        p.id = id;
        memcpy(&p.data, data, 8);
        return true;
    }
    else {
        return false;
    }
}

bool CAN::RawInterface::hasPacket()
{
    return serialInterface.available() >= 12;
}
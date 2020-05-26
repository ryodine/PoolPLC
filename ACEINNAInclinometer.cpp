#include "ACEINNAInclinometer.h"

#include "CANInterface.h"

bool Inclinometer::ACEINNAInclinometer::begin()
{
    canInterface.begin(CAN::SerialBaudrate::baud_115200,
                       CAN::CANBusBaudrate::kbps_250);
    constexpr byte src = 0x11;
    constexpr byte dest = 0x80;

    canInterface.write(CAN::J1939Message(src, dest, 65365, 10),
                       dest); // Set Output data rate to 10Hz (10=10Hz; 20=5Hz)
    canInterface.write(CAN::J1939Message(src, dest, 65366, 1),
                       dest); // Only use SSI2 (Inclination) data
    canInterface.write(CAN::J1939Message(src, dest, 65367, 2),
                       dest); // 2Hz digital low pass filter
    delay(100);
    // flush the buffer
    canInterface.flushBuffer();
    return true;
}
bool Inclinometer::ACEINNAInclinometer::hasData()
{
    return canInterface.hasPacket();
}
Eigen::Vector2d Inclinometer::ACEINNAInclinometer::getData()
{
    if (hasData()) {
        CAN::J1939Message m = canInterface.read();
        canInterface
            .flushBuffer(); // This will ensure that we aren't reading
                            // offset packets by clearing the buffer after every
                            // read. This could delete messages, but all we
                            // expect is telemetry, so this is ok
        if (m.CanID.getPGN() == 61481) {
            byte *data = m.data;

            unsigned long pitch = ((unsigned long)data[2]) << 16 |
                                  ((unsigned long)data[1]) << 8 |
                                  ((unsigned long)data[0]);
            unsigned long roll = ((unsigned long)data[5]) << 16 |
                                 ((unsigned long)data[4]) << 8 |
                                 ((unsigned long)data[3]);
            
            double pitch_adjusted = (pitch * (1.0 / 32768) - 250.0);
            double roll_adjusted = (roll * (1.0 / 32768) - 250.0);

            return Eigen::Vector2d(pitch_adjusted * PI / 180.0, roll_adjusted * PI / 180.0);
        }
        Serial.print("PGN was actually: ");
        Serial.println(m.CanID.getPGN());
    }

    return Eigen::Vector2d(-100.0, 100.0);
}
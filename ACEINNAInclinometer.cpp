#include "ACEINNAInclinometer.h"

#include "CANInterface.h"
#include "FaultHandling.h"

//! UNCOMMENT BELOW IF YOU WANT TO SET UP THE ACEINNA INCLINOMETER (FIRST TIME)
//#define PROVISION_CAN_ACEINNA_MODULE

bool Inclinometer::ACEINNAInclinometer::begin()
{
    canInterface.begin(CAN::SerialBaudrate::baud_115200,
                       CAN::CANBusBaudrate::kbps_250);

#ifdef PROVISION_CAN_ACEINNA_MODULE
    // define PROVISION_CAN_ACEINNA_MODULE to make the aceinna module be set up once
    ProvisionACEINNAInclinometer();
#endif

    canInterface.flushBuffer();
    delay(300);
    if (hasData())
        return true;

    return false;
}
bool Inclinometer::ACEINNAInclinometer::hasData()
{
    // TODO: pre-read and check if there is SSI2 data, then cache it!
    if (canInterface.hasPacket()) {
        CAN::J1939Message m = canInterface.read();
        canInterface
            .flushBuffer(); // This will ensure that we aren't reading
                            // offset packets by clearing the buffer after every
                            // read. This could delete messages, but all we
                            // expect is telemetry, so this is ok
        if (m.CanID.getPGN() == PGN_SSI2DATA) {
            byte *data = m.data;

            unsigned long pitch = ((unsigned long)data[2]) << 16 |
                                  ((unsigned long)data[1]) << 8 |
                                  ((unsigned long)data[0]);
            unsigned long roll = ((unsigned long)data[5]) << 16 |
                                 ((unsigned long)data[4]) << 8 |
                                 ((unsigned long)data[3]);

            double pitch_adjusted = (pitch * (1.0 / 32768) - 250.0);
            double roll_adjusted = (roll * (1.0 / 32768) - 250.0);

            if (pitch_adjusted > k_anglePlausibilityRange ||
                pitch_adjusted < -k_anglePlausibilityRange ||
                roll_adjusted > k_anglePlausibilityRange ||
                roll_adjusted < -k_anglePlausibilityRange) {
                Fault::Handler::instance()->setFaultCode(
                    Fault::INCL_IMPLAUS_READ);
            } else {
                Fault::Handler::instance()->unlatchFaultCode(
                    Fault::INCL_IMPLAUS_READ);
            }

            cachedAngles = Eigen::Vector2d(pitch_adjusted * PI / 180.0,
                                           roll_adjusted * PI / 180.0);
            hasDataCached = true;
            return true;
        }
    }
    return false;
}
Eigen::Vector2d Inclinometer::ACEINNAInclinometer::getData()
{
    hasDataCached = false;
    roll.addPoint(cachedAngles[0]);
    pitch.addPoint(cachedAngles[1]);
    return Eigen::Vector2d(roll.getAverage(), pitch.getAverage());
}

void Inclinometer::ACEINNAInclinometer::ProvisionACEINNAInclinometer()
{
    // Set Output data rate to 10Hz (10=10Hz; 20=5Hz)
    canInterface.write(
        CAN::J1939Message(k_sourceAddress, k_aceinnaAddress, PGN_ODR, 10),
        k_aceinnaAddress);

    // Only use SSI2 (Inclination) data
    canInterface.write(CAN::J1939Message(k_sourceAddress, k_aceinnaAddress,
                                         PGN_PERIODIC_DATA_TYPES, 1),
                       k_aceinnaAddress);

    // 2Hz digital low pass filter
    canInterface.write(CAN::J1939Message(k_sourceAddress, k_aceinnaAddress,
                                         PGN_LOW_PASS, 2, 2),
                       k_aceinnaAddress);

    // Save config to EEPROM
    canInterface.write(CAN::J1939Message(k_sourceAddress, k_aceinnaAddress,
                                         PGN_SAVE_EEPROM, 0, k_aceinnaAddress,
                                         1),
                       k_aceinnaAddress);
}
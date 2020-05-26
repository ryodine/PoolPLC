#include "CANSAEJ1939.h"

bool CAN::J1939Interface::j1939PeerToPeer(unsigned long lPGN)
{
    if (lPGN > 0 && lPGN <= 0xEFFF)
        return true;

    return false;
}

CAN::J1939Message CAN::J1939Interface::read()
{
    CAN::ExtendedCanDataPacket rawpacket;
    CAN::RawInterface::read(rawpacket);
    return CAN::J1939Message(rawpacket);
}

void CAN::J1939Interface::write(CAN::J1939Message p, byte dest)
{
    CAN::ExtendedCanDataPacket rawpacket;
    rawpacket.id = p.CanID.getID();
    if (j1939PeerToPeer(p.CanID.getPGN()) == true) {
        rawpacket.id = rawpacket.id & 0xFFFF00FF;
        rawpacket.id = rawpacket.id | ((unsigned long)dest << 8);
    }
    memcpy(&rawpacket.data, p.data, 8);
    CAN::RawInterface::write(rawpacket);
}
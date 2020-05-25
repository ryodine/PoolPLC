#include <Controllino.h>


#define PGN_RequestMessage                      0x00EA00


void resetSerial2IntoCfg() {
  Serial2.begin(9600);
  Serial2.print("+++");
  Serial2.println("AT+S=4");
  delay(100);
  Serial2.begin(19200);
  Serial2.print("+++");
  Serial2.println("AT+S=4");
    delay(100);

  Serial2.begin(38400);
  Serial2.print("+++");
  Serial2.println("AT+S=4");
    delay(100);

  Serial2.begin(57600);
  Serial2.print("+++");
  Serial2.println("AT+S=4");
    delay(100);

  Serial2.begin(115200);
  Serial2.print("+++");
  Serial2.println("AT+S=4");
    delay(100);

  Serial2.begin(115200);
  Serial2.print("+++");
}

void sprint() {
  while(Serial1.available()) {
    Serial.write(Serial1.read());
  }
}

void setup() {
  // put your setup code here, to run once:
  
  resetSerial2IntoCfg();
  Serial.begin(9600);
  delay(100);
  Serial.println("SPEED");
  Serial2.println("AT+C=15");
  delay(100);
  sprint();
  Serial2.println("AT+M=[0][1][1FFFFFFF]");
  delay(100);
  sprint();
  /*Serial2.println("AT+M=[1][1][1FFFFFFF]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[0][1][0CF02980]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[1][1][0CF02980]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[2][1][0CF02980]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[3][1][0CF02980]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[4][1][0CF02980]");
  delay(100);
  sprint();
  Serial2.println("AT+F=[5][1][0CF02980]");
  delay(100);
  sprint();*/
  Serial2.println("AT+Q");

  Serial.println("Setup done");
  delay(100);
  saeJ1939SetCommandLen1(0x11,0x80,65365,10); // ODR (10=10Hz; 50=2Hz)
  saeJ1939SetCommandLen1(0x11,0x80,65366,1); // ONLY SSI2
  saeJ1939SetCommandLen2(0x11,0x80,65367,2,2); // LPF
  
  //saeJ1939Request(0xFF,0x80,65242);
  byte buf[8] = {0x80, 50, 0, 0, 0, 0, 0, 0};
  //send(0x18FF5511, 1, 0, 8, buf);

  // Clear serial2 buffers
  while (Serial2.available()) Serial2.read();
}

long period = 50;
long last = 1000;
void loop() {
  /*if (millis() - last > period) {
    saeJ1939Request(0xFF,0x80,65242);
    last = millis();
  }*/
  // put your main code here, to run repeatedly:
  /*if (Serial.available()) {      // If anything comes in Serial (USB),
    Serial2.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }*/
  /*
  if (Serial2.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(Serial2.read());   // read it and send it out Serial (USB)
  }*/
  if (Serial2.available() >= 12) {
    
    byte buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    for (int i = 0; i < 12; i++) {
      buf[i] = Serial2.read();      
    }
    unsigned long id = 0;
    byte *data = buf+4;
    for (int i = 0; i < 4; i++) {
        id <<= 8;
        id += buf[i];
    }
    if (millis() - last > period) {
      if ((id & (long)0xFFFF00) >> 8 == 61481) {
        unsigned long pitch = ((unsigned long)data[2]) << 16 | ((unsigned long)data[1]) << 8 | ((unsigned long)data[0]);
        unsigned long roll = ((unsigned long)data[5]) << 16 | ((unsigned long)data[4]) << 8 | ((unsigned long)data[3]);

        double pitch_adjusted = (pitch * (1.0/32768) - 250.0);
        double roll_adjusted = (roll * (1.0/32768) - 250.0);

        Serial.print(pitch_adjusted);
        Serial.print("\t");
        Serial.println(roll_adjusted);
      
      } else {
        Serial.print("UNKNOWN PACKET PGN: 0x");
        Serial.println(id, HEX);
        for (int i = 0; i < 8; i++) {
          Serial.print(data[i], HEX);
          Serial.print(", ");
        }
        Serial.println();
      }
      last = millis();
    }
  }
}

bool j1939PeerToPeer(unsigned long lPGN)
{
  // Check the PGN 
  if(lPGN > 0 && lPGN <= 0xEFFF)
    return true;

  return false;

}

unsigned long saeJ1939Request(byte nSrcAddr, byte nDestAddr, unsigned long lPGN)
{
  unsigned char pgnPayload[8] = {lPGN & 0xFF, (lPGN >> 8) & 0xFF, (lPGN >> 16) & 0xFF, 0, 0, 0, 0, 0};
  return saeJ1939SEND(PGN_RequestMessage, 6, nSrcAddr, nDestAddr, 0, pgnPayload, 8);  // Transmit the message
}

unsigned long saeJ1939SetCommandLen1(byte nSrcAddr, byte nDestAddr, unsigned long lPGN, byte value)
{
  unsigned char pgnPayload[8] = {nDestAddr, value, 0, 0, 0, 0, 0, 0};
  return saeJ1939SEND(lPGN, 6, nSrcAddr, 0xFF, 0, pgnPayload, 8);  // Transmit the message
}

unsigned long saeJ1939SetCommandLen2(byte nSrcAddr, byte nDestAddr, unsigned long lPGN, byte value, byte value2)
{
  unsigned char pgnPayload[8] = {nDestAddr, value, value2, 0, 0, 0, 0, 0};
  return saeJ1939SEND(lPGN, 6, nSrcAddr, 0xFF, 0, pgnPayload, 8);  // Transmit the message
}

unsigned long saeJ1939SEND(unsigned long lPGN, byte nPriority, byte nSrcAddr, byte nDestAddr, unsigned char rtrBit, unsigned char* nData, int nDataLen)
{
  // Declarations
  unsigned long lID = ((unsigned long)nPriority << 26) + (lPGN << 8) + (unsigned long)nSrcAddr;
  
  // If PGN represents a peer-to-peer, add destination address to the ID
  if(j1939PeerToPeer(lPGN) == true) {
    lID = lID & 0xFFFF00FF;
    lID = lID | ((unsigned long)nDestAddr << 8);
  }

  return send(lID, 1, rtrBit, nDataLen, nData);
}

unsigned char send(unsigned long id, unsigned char ext, unsigned char rtrBit, unsigned char len, unsigned char *buf)
{
    unsigned char dta[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    dta[0] = id>>24;        // id3
    dta[1] = id>>16&0xff;   // id2
    dta[2] = id>>8&0xff;    // id1
    dta[3] = id&0xff;       // id0
    
    dta[4] = ext;
    dta[5] = rtrBit;
    
    for(int i=0; i<len; i++)
    {
        dta[6+i] = buf[i];
    }
    
    for(int i=0; i<14; i++)
    {
        Serial2.write(dta[i]);
    }
}


// 0: no data
// 1: get data
unsigned char recv(unsigned long *id, unsigned char *buf)
{
    if(Serial2.available())
    {
        unsigned long timer_s = millis();
        
        int len = 0;
        unsigned char dta[20];
        
        while(1)
        {
            while(Serial2.available())
            {
                dta[len++] = Serial2.read();
    if(len == 12)
                    break;
                timer_s = millis();
              if((millis()-timer_s) > 10)
                    return 0; // Reading 12 bytes should be faster than 10ms, abort if it takes longer, we loose the partial message in this case
            }
            
            if(len == 12) // Just to be sure, must be 12 here
            {
                unsigned long __id = 0;
                
                for(int i=0; i<4; i++) // Store the id of the sender
                {
                    __id <<= 8;
                    __id += dta[i];
                }
                
                *id = __id;
                
                for(int i=0; i<8; i++) // Store the message in the buffer
                {
                    buf[i] = dta[i+4];
                }
                return 1;
            }
        }
    }
    
    return 0;
}

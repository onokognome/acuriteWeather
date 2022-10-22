#include <RadioHead.h>
//#include <radio_config_Si4460.h>
#include <RHCRC.h>
#include <RHDatagram.h>
//#include <RHGenericDriver.h>
//#include <RHGenericSPI.h>
//#include <RHHardwareSPI.h>
//#include <RHMesh.h>
//#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
//#include <RHRouter.h>
#include <RHSoftwareSPI.h>
//#include <RHSPIDriver.h>
//#include <RHTcpProtocol.h>
//#include <RH_ASK.h>
//#include <RH_CC110.h>
//#include <RH_MRF89.h>
//#include <RH_NRF24.h>
//#include <RH_NRF51.h>
//#include <RH_NRF905.h>
//#include <RH_RF22.h>
//#include <RH_RF24.h>
#include <RH_RF69.h>
//#include <RH_RF95.h>
#include <RH_Serial.h>
//#include <RH_TCP.h>

#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"


RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  // put your setup code here, to run once:

 Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  rf69.setModeRx();
}

void loop() {
  // put your main code here, to run repeatedly:

  

}

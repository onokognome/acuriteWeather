

#include <SPI.h>
#include "RH_RF69.h"


#define RFM69_CS 10
#define RFM69_RST 9
#define RFM69_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF69 rf69();

void setup() {
  // put your setup code here, to run once:
 
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}

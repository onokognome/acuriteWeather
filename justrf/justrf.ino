#include<Arduino.h>
#include<SPI.h>

//Radio Parameters
#define RF_Freq 433               //frequence in Mhz

// The crystal oscillator frequency of the RF69 module
#define RH_RF69_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF69_FXOSC / 2^^19
#define RH_RF69_FSTEP  (RH_RF69_FXOSC / 524288)

#define My_FRF (uint32_t)((RF_Freq * 1000000.0) / RH_RF69_FSTEP)

#define RegOpModeAddr     0x01
#define RegOpModeValue    0x10    // RX mode
#define RegDataModulAddr  0x02
#define RegDataModulValue 0x48    // Continuous w/sync, OOK. 0xC8 is no sync
#define RegBitrateMSBAddr 0x03
#define RegBitrateMSBValue 0x4E
#define RegBitrateLSBAddr 0x04
#define RegBitrateLSBValue  0x20  //1600Hz
#define RegFrFMsbAddr     0x07
#define RegFrFMsbValue    ((My_FRF >> 16) & 0xff)
#define RegFrfMidAddr     0x08
#define RegFrfMidValue    ((My_FRF >> 8) & 0xff)
#define RefFrfLsbAddr     0x09
#define RefFrfLsbValue    (My_FRF & 0xff)

#define RFM69_CS          4
#define RFM69_INT         3
#define RFM69_RST         2

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.print(F("Hello! ST77xx TFT Display"));
  
 // initialize radio
SPI.begin();
  pinMode(RFM69_CS, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT, INPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_CS, HIGH);
//  attachInterrupt(digitalPinToInterrupt(RFM69_INT), RM69_vect, RISING);

  //reset radio
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  //initialize ook
  rfm69_write(RegOpModeAddr, RegOpModeValue);
  rfm69_write (RegDataModulAddr, RegDataModulValue);
  rfm69_write( RegBitrateMSBAddr,RegBitrateMSBValue);
  rfm69_write( RegBitrateLSBAddr, RegBitrateLSBValue);
  rfm69_write( RegFrFMsbAddr,RegFrFMsbValue);
  rfm69_write( RegFrfMidAddr,RegFrfMidValue);
  rfm69_write( RefFrfLsbAddr,RefFrfLsbValue);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void rfm69_write(byte address, byte value) {
 unsigned int in_value;
 
SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(RFM69_CS, LOW);
  Serial.print("r"); 
  in_value = SPI.transfer16(((address | 0x80) << 8) | value);
//  in_value = SPI.transfer(address);
  Serial.println(in_value, DEC);
  digitalWrite(RFM69_CS, HIGH);
  SPI.endTransaction();
  
}

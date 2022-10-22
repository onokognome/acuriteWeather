
/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <RTClib.h>

 
#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
  #define TFT_CS         14
  #define TFT_RST        15
  #define TFT_DC         32

#elif defined(ESP8266)
  #define TFT_CS         4
  #define TFT_RST        16                                            
  #define TFT_DC         5

#else
  // For the breakout board, you can use any 2 or 3 pins.
  // These pins will also work for the 1.8" TFT shield.
  #define TFT_CS        10
  #define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         8
#endif




// OPTION 1 (recommended) is to use the HARDWARE SPI pins, which are unique
// to each board and not reassignable. For Arduino Uno: MOSI = pin 11 and
// SCLK = pin 13. This is the fastest mode of operation and is required if
// using the breakout board's microSD card.

// For 1.44" and 1.8" TFT with ST7735 use:
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// For 1.14", 1.3", 1.54", and 2.0" TFT with ST7789:
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);




// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out

// For ST7735-based displays, we will use this call
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// OR for the ST7789-based displays, we will use this call
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


volatile unsigned int   counter = 0;
unsigned int            last_counter = 0;
volatile unsigned long  rf_counter = 0;
unsigned long           loop_counter = 0;

volatile unsigned int   rf_data;
volatile bool           rf_data_present = false;
volatile unsigned long  rf_counter_latch = 0L;

int8_t hour = 6;
int8_t minute = 14;
char time_str[6] = "00:00";
#define ST_HR1 0
#define ST_HR2 1
#define ST_MIN1 3
#define ST_MIN2 4

RTC_DS3231 rtc;

#define TCNT1_value 64*1024-16000000/8/10000
ISR(TIMER1_OVF_vect) {
  rf_counter++;
  TCNT1 = TCNT1_value;
  if (rf_counter > 10000L) { counter++; rf_counter = 0;}
  if (!(rf_counter % 10)) RFM69_vect();
}


//Radio Parameters
#define RF_Freq 433               //frequence in Mhz

// The crystal oscillator frequency of the RF69 module
#define RH_RF69_FXOSC 32000000L

// The Frequency Synthesizer step = RH_RF69_FXOSC / 2^^19
#define RH_RF69_FSTEP   61 //  (RH_RF69_FXOSC / 524288)

#define My_FRF (uint32_t)((RF_Freq * 1000000L) / RH_RF69_FSTEP)

#define MY_BITRATE 4800
#define BITRATE (uint16_t)(RH_RF69_FXOSC/MY_BITRATE)

#define RegOpModeAddr     0x01
#define RegOpModeValue    0x10    // RX mode
#define RegDataModulAddr  0x02
#define RegDataModulValue 0x69    // 0x48 Continuous w/sync, OOK. 0x68 is no sync
#define RegBitrateMSBAddr 0x03
#define RegBitrateMSBValue ((BITRATE >> 8) & 0xff)
#define RegBitrateLSBAddr 0x04
#define RegBitrateLSBValue (BITRATE & 0xff)
#define RegFrFMsbAddr     0x07
#define RegFrFMsbValue    ((My_FRF >> 16) & 0xff)
#define RegFrfMidAddr     0x08
#define RegFrfMidValue    ((My_FRF >> 8) & 0xff)
#define RefFrfLsbAddr     0x09
#define RefFrfLsbValue    (My_FRF & 0xff)
#define RegOokPeakAddr    0x1b
#define RegOokPeakValue   0x0      //fixed
#define RegOokFixAddr     0x1d
#define RegOokFixValue    0x0F
#define RegLnaAddr       0x18
#define RegLnaValue       0x02
#define RegRxBwAddr       0x29
#define RegRxBwValue      0x42   

#define RFM69_CS          4
#define RFM69_INT         3
#define RFM69_RST         2
#define RFM69_DATA        6

volatile unsigned int vect_state = 0;
unsigned int count_zero = 0;
unsigned int count_one = 0;
volatile unsigned int one_ready = 0;
volatile unsigned int zero_ready = 0;


void bit_machine() {
  static unsigned int counting_zero = 1;
  
  if (counting_zero) {
    if (rf_data == 0) count_zero++;
    else {
      counting_zero = 0;
      count_one = 1;
//      zero_ready = 1;
    }
  } else {
    if (rf_data == 1) count_one++;
    else {
      if (count_one >= 8) {
        counting_zero = 1; 
//        count_zero = 1;
        one_ready = 1;
        zero_ready = 1;
      } else {
        counting_zero = 1;
      }
    }
  }
}

void RFM69_vect() {
  rf_data = digitalRead(RFM69_DATA);
  bit_machine();
//  if (rf_data == 0) {
//    vect_state = 0;
//    return;
//  }
  
//  vect_state++;
  // rf_data_present = true;
//  if (vect_state == 12) { 
//    rf_data_present = true;
//    vect_state = 0;
//  }
}



void setup(void) {
  Serial.begin(115200);
  Serial.print(F("Hello! ST77xx TFT Display"));

  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, time is not correct!");
  }

   // OR use this initializer (uncomment) if using a 1.14" 240x135 TFT:
  tft.init(135, 240);           // Init ST7789 240x135

  // set landscape mode
  tft.setRotation( 3 );

   // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  //tft.setSPISpeed(40000000);

  Serial.println(F("Initialized"));

  tft.setCursor(0,0);
  tft.println("done");
  delay(1000); 

   tft.fillScreen(ST77XX_BLACK);
   tft.setTextSize(6);
   tft.setTextColor(ST77XX_BLUE, ST77XX_YELLOW);

 noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 64*1014-16000000/8/100000;        // preload timer 65536-16MHz/prescaler/desiredFreqHz
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();  


  // initialize radio
// SPI.begin();
  pinMode(RFM69_CS, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
//  pinMode(RFM69_INT, INPUT);   // DCLK in Continuous mode
  pinMode(RFM69_DATA, INPUT);  // Data in Continous mode
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_CS, HIGH);
//  attachInterrupt(digitalPinToInterrupt(RFM69_INT), RFM69_vect, RISING);

  //reset radio
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  //initialize ook
 
  rfm69_write (RegDataModulAddr, RegDataModulValue);
//  rfm69_write( RegBitrateMSBAddr,RegBitrateMSBValue);
//  rfm69_write( RegBitrateLSBAddr, RegBitrateLSBValue);
  rfm69_write( RegFrFMsbAddr,RegFrFMsbValue);
  rfm69_write( RegFrfMidAddr,RegFrfMidValue);
  rfm69_write( RefFrfLsbAddr,RefFrfLsbValue);
//  rfm69_write( RegRxBwAddr, RegRxBwValue);
  rfm69_write( RegLnaAddr, RegLnaValue);
//  rfm69_write( RegOokPeakAddr, RegOokPeakValue);
//  rfm69_write( RegOokFixAddr, RegOokFixValue);
   rfm69_write(RegOpModeAddr, RegOpModeValue);

   delay(100);
   Serial.print("RFM69 mode:"); Serial.println(rfm69_read(RegOpModeAddr), HEX);
   Serial.print("RFM69 frfmsb:");Serial.println(rfm69_read(RegFrFMsbAddr), HEX);
   Serial.print("RFM69 frfmid:");Serial.println(rfm69_read(RegFrfMidAddr), HEX);
   Serial.print("RFM69 frflsb:");Serial.println(rfm69_read(RefFrfLsbAddr), HEX);
   
   delay(3000);
  
 }

void loop() {

  loop_counter++;
//  if(rf_data_present == true) {
//    Serial.print(rf_data  ? "1" : "0");
//    Serial.print(rf_counter, DEC);
//    noInterrupts();
//    rf_data_present = false;
//    vect_state = 0;
//    interrupts();
// }

    if (zero_ready) {
//      if (count_zero < 8)
//        Serial.print("L");Serial.print(count_zero, DEC);
      zero_ready = 0;
    }

    if(one_ready) {
 //     if (count_one > 5)
        Serial.print("H");Serial.print(count_one, DEC);
      one_ready = 0;
    }
 
  
  if (last_counter != counter) {
    tft.setCursor(0,0);
    tft.print(counter, DEC); tft.print (" "); tft.print(loop_counter/1000, DEC);
    Serial.print("*"); Serial.println(rf_counter, DEC);
    last_counter = counter;
    loop_counter = 0;
  }
  
   if (counter >= 60) {
      counter = 0;
      DateTime now = rtc.now();

      tft.setCursor(0,50);
      hour = (now.hour() > 12) ? now.hour() - 12 : now.hour();
      tft.print(hour, DEC); tft.print(":"); tft.print(now.minute(),DEC);
   }

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

byte  rfm69_read(byte address) {
  byte value;
  
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(RFM69_CS, LOW);
  value = SPI.transfer16(address<<8);
  digitalWrite(RFM69_CS, HIGH);
  SPI.endTransaction();
  return value;
}



/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
    
#define LED_PIN 13

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

bool isLEDOn = false;

// We start with a very low sample rate just to get started.
int nFreq = 10;  // 10 Hz - will be 1000 Hz
int msecSamplePeriod = 1000/nFreq;  // Or 100 milliseconds - will be 1 millisecond

  uint8_t strName[8] = "PL21234";
/**************************************************************************/
void setupBLE(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit AT Command Example"));
  Serial.println(F("-------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  ble.setAdvData(strName, 7);
    /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
}


void setup() {
  setupBLE();
  pinMode(LED_PIN, OUTPUT);
  startTimer(nFreq);
}

void dataAnalysis(void); 

void loop() {
    dataAnalysis();
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void dataCollection(void);

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    dataCollection();
    digitalWrite(LED_PIN, isLEDOn);
    isLEDOn = !isLEDOn;
  }
}

///////////////////////////////////////////////////////////////////////////////

int data[256][3] = {0};
volatile int idxForCollection = 0;
volatile int idxForAnalysis = 0;
volatile bool isTriggered = false;

void dataCollection(void)
{
  data[idxForCollection][0] = analogRead(A0);
  data[idxForCollection][1] = analogRead(A1);
  data[idxForCollection][2] = analogRead(A3);
  idxForCollection++;
  if ((idxForCollection % 64) == 0) {
    isTriggered = true;
    idxForAnalysis = idxForCollection - 64;
    Serial.print("Triggered for analysis: ");
    Serial.println(idxForAnalysis);
  }
  if ((idxForCollection % 256) == 0) {
    idxForCollection = 0;
  }
}

 
void connect(const char *strName)
{
  String strCmd = "AT+GAPDEVNAME=" + String(strName) + "_CTE";
  Serial.print("Advertizing "); Serial.println(strCmd);
  ble.sendCommandCheckOK(strCmd.c_str());
  ble.sendCommandCheckOK("ATZ");
  delay(2000);
  String strCmdWaiting = "AT+GAPDEVNAME=" + String(strName);
  ble.sendCommandCheckOK(strCmdWaiting.c_str());
  ble.sendCommandCheckOK("ATZ");
  delay(2000);
}

void report(int idx) {
  Serial.print("Triggered for report: ");
  Serial.print(data[idx][0]); Serial.print(", ");
  Serial.print(data[idx][1]); Serial.print(", ");
  Serial.println(data[idx][2]);

  int valueX = (((data[idx][0]-512)*200)/1024);
  int valueY = (((data[idx][1]-512)*200)/1024);
  int valueZ = (((data[idx][2]-512)*200)/1024);
  
  //Serial.print("x "); Serial.println(valueX);
  //Serial.print("y "); Serial.println(valueY); 
  //Serial.print("z "); Serial.println(valueZ);

 
  // Send bluetooth msg
  ble.print("AT+BLEFriend="); 
  if (valueX > 44) {
  int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueX));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());
  }
  if (valueX < -44) {
   
  int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueX));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());
  }
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send X?"));
  }   
  ble.print("AT+BLEFriend="); 
  if (valueY > 44) {
    
  int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueY));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());
  }
  if (valueY < -44) {
   
  int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueY));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());;
  }
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send Y?"));
  }  
  ble.print("AT+BLEFriend="); 
  if (valueZ > 44) {
   
  int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueZ));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());
  }
  if (valueZ < -44) {
    int nPlayer = random(10, 50);
  String strAd = String("Player_") + String(nPlayer).c_str();
  strAd = strAd + "_" + String(abs(valueZ));
  Serial.print("Advertizing: "); Serial.println(strAd.c_str());
  connect(strAd.c_str());
  }
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send Z?"));
  }
}


void dataAnalysis(void) {
  if (!isTriggered) {
    // We will wait for an interval that should allow the
    // next 64 samples to have been collected.
    int nSamplesLeftToCollect = 64 - (idxForCollection % 64); 
    Serial.print("Samples to collect: ");
    Serial.println(nSamplesLeftToCollect);
    // In our working version, nFreq will be 1, so that
    // the following call will delay for nDelay milliseconds;
    delay(nSamplesLeftToCollect * msecSamplePeriod);
    while (!isTriggered) {
      Serial.println("Not ready yet!");    
      delay(5 * msecSamplePeriod);
    }
  }
  isTriggered = false;

  // Look for event.
  int idx = idxForAnalysis;
  for (int i = idx; i < idx + 8; i++) {
    Serial.print(data[i][0]);
    Serial.print(" ");
  }
  Serial.println(" - [ X samples]");
  // Our test threshold is very low. 
  // Note: we should be testing =below 512 as well (0 to -200 g)
  for (int i = idx; i < idx + 64; i++) {
    if (
      (data[i][0] > 525)
      || (data[i][1] > 525)
      || (data[i][2] > 525)
        ) {
          report(i);
          break;
    }
  }
}


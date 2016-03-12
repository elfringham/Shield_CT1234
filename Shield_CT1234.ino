/*
 EmonTx Shield 4 x CT example
 
  An example sketch for the emontx Arduino shield module for
 CT only electricity monitoring.
 
 Part of the openenergymonitor.org project
 Licence: GNU GPL V3
 
 Authors: Glyn Hudson, Trystan Lea
 Builds upon Ciseco SRFSPI library and Arduino
 
 emonTx documentation: 	http://openenergymonitor.org/emon/modules/emontxshield/
 emonTx firmware code explination: http://openenergymonitor.org/emon/modules/emontx/firmware
 emonTx calibration instructions: http://openenergymonitor.org/emon/modules/emontx/firmware/calibration

 THIS SKETCH REQUIRES:

 Libraries in the standard arduino libraries folder:
	- SRFSPI		https://github.com/CisecoPlc/SRFSPI
	- EmonLib		https://github.com/openenergymonitor/EmonLib.git

 Other files in project directory (should appear in the arduino tabs above)
	- emontx_lib.ino
 
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#define FILTERSETTLETIME 10000                                          //  Time (ms) to allow the filters to settle before sending data

const int CT1 = 1; 
const int CT2 = 0;                                                      // Set to 0 to disable 
const int CT3 = 0;
const int CT4 = 0;

#include <SPI.h>
#include <SRFSPI.h>
#include "EmonLib.h"
EnergyMonitor ct1,ct2,ct3, ct4;                                              // Create  instances for each CT channel

typedef struct {
  int power1;
  int power2;
  int power3;
  int power4;
  float temp;
} PayloadTX;

PayloadTX emontx;
uint8_t PANID[2];

const int LEDpin = 9;                                                   // On-board emonTx LED 

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

boolean settled = false;

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setup() 
{
  uint8_t rxbuf[64];
  char msg[48];
  unsigned int i, retries = 10;
  Serial.begin(9600);
  Serial.println("EmonTX Shield CT1234 with SRF"); 
  Serial.println("OpenEnergyMonitor.org");
             
  if (CT1) ct1.current(1, 60.606);                                     // Setup emonTX CT channel (channel, calibration)
  if (CT2) ct2.current(2, 60.606);                                     // Calibration factor = CT ratio / burden resistance
  if (CT3) ct3.current(3, 60.606); 
  if (CT4) ct4.current(4, 60.606); 

 // emonTx Shield Calibration = (100A / 0.05A) / 33 Ohms

  SRF.init(10);
  PANID[0] = 'P';
  PANID[1] = 'M';

  pinMode(LEDpin, OUTPUT);                                              // Setup indicator LED
  digitalWrite(LEDpin, HIGH);
  while(retries--) {
    i = 0;
    delay(1100);
    SRF.write((uint8_t *)"+++", 3);
    delay(1000);
    while(!SRF.available() && i < 1000) {
      delay(10);
      i++;
    }
    i = 0;
    while(SRF.available()) {
      rxbuf[i % 64] = SRF.read();
      i++;
    }
    if ((i == 3) && rxbuf[0] == 'O' && rxbuf[1] == 'K') {
      //Serial.println("Got OK back successfully");
      SRF.write((uint8_t *)"ATMY\r", 5);
      i = 0;
      while(!SRF.available() && i < 1000) {
        delay(10);
        i++;
      }
      i = 0;
      while(SRF.available()) {
        rxbuf[i % 64] = SRF.read();
        i++;
        //Serial.write(rxbuf[i-1]);
        //if (rxbuf[i-1] == '\r') Serial.write('\n');
      }
      if ((i == 6) && rxbuf[3] == 'O' && rxbuf[4] == 'K') {
        snprintf(msg, 48, "ATMY returned OK and set PANID to '%c%c'", rxbuf[0], rxbuf[1]);
        Serial.println(msg);
        PANID[0] = rxbuf[0];
        PANID[1] = rxbuf[1];
        retries = 0;
      } else {
        snprintf(msg, 48, "Got back i = %d", i);
        Serial.println(msg);
      }
    } else {
      snprintf(msg, 48, "Got back i = %d and response '%c' '%c'", i, rxbuf[0], rxbuf[1]);
      Serial.println(msg);
    }
    SRF.write((uint8_t *)"ATDN\r", 5);
    delay(100);
    i = 0;
    while(SRF.available()) {
      rxbuf[i % 64] = SRF.read();
      i++;
    }
  }
  
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();
  // set the resolution to 11 bits (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 11);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
}

void check_SRF(void)
{
  uint8_t rxbuf[8];
  unsigned int i = 0;
  delay(100);
  SRF.write((uint8_t *)"+++", 3);
  delay(1000);
  while(!SRF.available() && i < 1000) {
    delay(10);
    i++;
  }
  i = 0;
  while(SRF.available()) {
    rxbuf[i % 8] = SRF.read();
    i++;
  }
  if ((i != 3) || rxbuf[0] != 'O' || rxbuf[1] != 'K') {
    Serial.println("Could not get OK from SRF");
  }
  SRF.write((uint8_t *)"ATDN\r", 5);
  i = 0;
  while(!SRF.available() && i < 1000) {
    delay(10);
    i++;
  }
  i = 0;
  while(SRF.available()) {
    rxbuf[i % 8] = SRF.read();
    i++;
  }
}

int loop_count;

void loop() 
{
  if (CT1) {
    Serial.print("CT1 ");
    emontx.power1 = ct1.calcIrms(1480) * 240.0;                         //ct.calcIrms(number of wavelengths sample)*AC RMS voltage
    Serial.print(emontx.power1);                                         
  }
  
  if (CT2) {
    Serial.print("CT2 ");
    emontx.power2 = ct2.calcIrms(1480) * 240.0;
    Serial.print(" "); Serial.print(emontx.power2);
  } 

  if (CT3) {
    Serial.print("CT3 ");
    emontx.power3 = ct3.calcIrms(1480) * 240.0;
    Serial.print(" "); Serial.print(emontx.power3);
  } 
  
   if (CT4) {
    Serial.print("CT4 ");
    emontx.power4 = ct4.calcIrms(1480) * 240.0;
    Serial.print(" "); Serial.print(emontx.power4);
  } 
  
  
  Serial.println(); delay(100);

  // because millis() returns to zero after 50 days ! 
  if (!settled && millis() > FILTERSETTLETIME) settled = true;

  if (settled)                                                            // send data only after filters have settled
  {
    send_rf_data();                                                       // *SEND RF DATA* - see emontx_lib
    sensors.requestTemperatures();
    delay(1000);
    emontx.temp = sensors.getTempC(insideThermometer);
    Serial.print("Temperature measured is ");
    Serial.println(emontx.temp);
    send_temp_data();
    digitalWrite(LEDpin, HIGH); delay(2); digitalWrite(LEDpin, LOW);      // flash LED
    delay(1000);                                                          // delay between readings in ms
    check_SRF();
  }
}


// Two DS18B20 sensors are connected to one digital 
// pin using one 4.7k pull-up resistor. Particularly,
// each sensor has a unique 64-bit address (to get it, 
// see DS18B20_get_address) that enables to use 1-Wire 
// communication bus system or protocol designed by 
// Dallas Semiconductors. This code is used in the 
// main Controllino program: "all_together.ino".

#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Addresses of two DS18B20s
uint8_t sensor1[8] = { 0x28, 0x7F, 0x17, 0xAC, 0x13, 0x19, 0x01, 0x9A };
uint8_t sensor2[8] = { 0x28, 0xFF, 0xC8, 0xC2, 0xC1, 0x16, 0x04, 0xB5 };

void setup(void){
  Serial.begin(9600);
  sensors.begin();
}

void loop(void){
  sensors.requestTemperatures();
  
  Serial.print("Temp_1 (°C): ");
  printTemperature(sensor1);
  
  Serial.print("Temp_2 (°C): ");
  printTemperature(sensor2);
  
  Serial.println();
}

void printTemperature(DeviceAddress deviceAddress){
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print(tempC);
  Serial.print("\t");
}

// REFERENCES:
// 1. Original code: https://lastminuteengineers.com/multiple-ds18b20-arduino-tutorial/
// 2. Datasheet: https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

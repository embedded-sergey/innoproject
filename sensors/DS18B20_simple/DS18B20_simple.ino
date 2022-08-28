// Two DS18B20 sensors are connected independently
// to different digital pins using two 4.7k pull-up 
// resistors. But, because this approach seriously 
// slows down calculations (i.e. +750 ms per sensor),
// we decided to use the 1-Wire communication protocol
// instead (see DS18B20_one_wire.ino). Nevertheless,
// this code works as well will be here...

#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS_1 2
#define ONE_WIRE_BUS_2 8

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire1(ONE_WIRE_BUS_1);  
OneWire oneWire2(ONE_WIRE_BUS_2); 
 
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);

void setup(void){
  Serial.begin(9600);
  sensor1.begin();  // Start up the library
  sensor2.begin();  // Start up the library
}

void loop(void){ 
  // Send the command to get temperatures
  sensor1.requestTemperatures(); 
  sensor2.requestTemperatures();
  
  //print the temperature in Celsius
  Serial.print("Temp_1 (°C): ");
  Serial.print(sensor1.getTempCByIndex(0));
  Serial.print("\t");
  Serial.print("Temp_2 (°C): ");
  Serial.println(sensor2.getTempCByIndex(0)); 
}

// REFERENCES:
// 1. Original code: https://lastminuteengineers.com/ds18b20-arduino-tutorial/
// 2. Datasheet: https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

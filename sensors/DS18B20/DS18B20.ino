#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS_1 2
#define ONE_WIRE_BUS_2 8
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire1(ONE_WIRE_BUS_1);  
OneWire oneWire2(ONE_WIRE_BUS_2);  
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);

void setup(void)
{
  sensors1.begin();  // Start up the library
  sensors2.begin();  // Start up the library
  Serial.begin(9600);
}

void loop(void)
{ 
  // Send the command to get temperatures
  sensors1.requestTemperatures(); 
  sensors2.requestTemperatures();
  
  //print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(sensors1.getTempCByIndex(0));
  Serial.print("°C  |  ");
  Serial.print(sensors2.getTempCByIndex(0));
  Serial.print("°C");  
  Serial.println();
}

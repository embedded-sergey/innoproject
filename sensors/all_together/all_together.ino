#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const byte ph_sensor_pin = A0;             // Analog 0    X1 
const byte tds_sensor_pin = A1;            // Analog 1    X1
const byte modbus_sensor_pin = A2;         // Analog 2    X1

const byte temperature_sensor_1_pin = 2;   // Digital 0   X1   
const byte temperature_sensor_2_pin = 3;   // Digital 1   X1
const byte water_level_trig_1_pin = 4;     // Digital 2   X1
const byte water_level_trig_2_pin = 5;     // Digital 3   X1
const byte water_level_echo_1_pin = 6;     // Digital 4   X1
const byte water_level_echo_2_pin = 7;     // Digital 5   X1
const byte buzzer_pin = 8;                 // Digital 6   X2

const byte water_level_pump_1_pin = 22;    // Relay 0     R0
const byte water_level_pump_2_pin = 23;    // Relay 1     R1
const byte water_mixing_pump_1_pin = 24;   // Relay 2     R2
const byte water_mixing_pump_2_pin = 25;   // Relay 3     R3
const byte water_rack_pump_1_pin = 26;     // Relay 4     R4
const byte water_rack_pump_2_pin = 27;     // Relay 5     R5
const byte led_light_1_pin = 28;           // Relay 6     R6
const byte led_light_2_pin = 29;           // Relay 7 .   R8
const byte water_heater_pin = 30;          // Relay 8     R7
const byte nutrient_dose_pump_pin = 31;    // Relay 9     R9

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire1(temperature_sensor_1_pin);
OneWire oneWire2(temperature_sensor_2_pin);
// Pass oneWire reference to DallasTemperature library
DallasTemperature temperature_sensor_1(&oneWire1);
DallasTemperature temperature_sensor_2(&oneWire2);

void setup(void){
  temperature_sensor_1.begin();  // Start up the library
  temperature_sensor_2.begin();  // Start up the library
  Serial.begin(9600);
}

void loop(void)
{ 
  // Send the command to get temperatures
  temperature_sensor_1.requestTemperatures();
  temperature_sensor_2.requestTemperatures();
  
  //print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(temperature_sensor_1.getTempCByIndex(0));
  Serial.print("°C  |  ");
  Serial.print(temperature_sensor_2.getTempCByIndex(0));
  Serial.print("°C");  
  Serial.println();
}

 

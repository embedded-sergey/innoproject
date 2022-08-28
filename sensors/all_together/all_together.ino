////////////////////////
// REQUIRED LIBRARIES //
////////////////////////
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

////////////////
// PINOUT MAP // 
////////////////
const byte ph_sensor_pin = A0;           // Analog 0    X1 
const byte tds_sensor_pin = A1;          // Analog 1    X1
const byte modbus_sensor_pin = A2;       // Analog 2    X1

const byte water_temp_pin = 2;           // Digital 0   X1
const byte water_level_trig_1_pin = 3;   // Digital 1   X1
const byte water_level_trig_2_pin = 4;   // Digital 2   X1
const byte water_level_echo_1_pin = 5;   // Digital 3   X1
const byte water_level_echo_2_pin = 6;   // Digital 4   X1
const byte buzzer_pin = 7;               // Digital 5   X2

const byte water_level_pump_1_pin = 22;  // Relay 0     R0
const byte water_level_pump_2_pin = 23;  // Relay 1     R1
const byte water_mixing_pump_1_pin = 24; // Relay 2     R2
const byte water_mixing_pump_2_pin = 25; // Relay 3     R3
const byte water_rack_pump_1_pin = 26;   // Relay 4     R4
const byte water_rack_pump_2_pin = 27;   // Relay 5     R5
const byte led_light_1_pin = 28;         // Relay 6     R6
const byte led_light_2_pin = 29;         // Relay 7     R8
const byte water_heater_pin = 30;        // Relay 8     R7
const byte nutrient_dose_pump_pin = 31;  // Relay 9     R9

/////////////////////////////////
// CONFIGURATIONS for DS18B20s //
///////////////////////////////// 
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(water_temp_pin);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// Insert addresses (to get, see DS18B20_get_address):
uint8_t water_temp_1_address[8] = { 0x28, 0x7F, 0x17, 0xAC, 0x13, 0x19, 0x01, 0x9A };
uint8_t water_temp_2_address[8] = { 0x28, 0xFF, 0xC8, 0xC2, 0xC1, 0x16, 0x04, 0xB5 };

void setup(void){
  Serial.begin(9600);
  sensors.begin(); // Start up the library
}

void loop(void){ 
  // Implementation code for Water Temperature sensors
  sensors.requestTemperatures();
  Serial.print("Temp_1 (°C): ");
  float water_temp_1 = sensors.getTempC(water_temp_1_address);
  Serial.print(water_temp_1);
  Serial.print("\t");
  Serial.print("Temp_2 (°C): ");
  float water_temp_2 = sensors.getTempC(water_temp_2_address);
  Serial.print(water_temp_2);
  Serial.println();
}

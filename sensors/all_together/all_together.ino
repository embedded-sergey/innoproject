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
const byte water_level_echo_1_pin = 4;   // Digital 2   X1
const byte water_level_trig_2_pin = 5;   // Digital 3   X1
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

///////////////////////////
// SENSOR CONFIGURATIONS //
///////////////////////////
// WATER TEMPERATURE
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(water_temp_pin);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// Insert addresses (to get, see DS18B20_get_address):
uint8_t water_temp_1_address[8] = { 0x28, 0x7F, 0x17, 0xAC, 0x13, 0x19, 0x01, 0x9A };
uint8_t water_temp_2_address[8] = { 0x28, 0xFF, 0xC8, 0xC2, 0xC1, 0x16, 0x04, 0xB5 };

// ELECTRIC CONDUCTIVITY
// No calibration required for EC. The electrical 
// conductivity of an aqueous solution increases with 
// temperature significantly: about 2 per Celsius.
const float a = 0.020; 


void setup(void){
  Serial.begin(9600);
  sensors.begin(); // Start up the library
}

void loop(void){ 
  ///////////////////////////////
  // Water Temperature sensors //
  /////////////////////////////// 
  sensors.requestTemperatures();
  Serial.print("Water_temp_1 (°C): ");
  float water_temp_1 = sensors.getTempC(water_temp_1_address);
  Serial.print(water_temp_1);
  Serial.print("\t");
  Serial.print("Water_temp_2 (°C): ");
  float water_temp_2 = sensors.getTempC(water_temp_2_address);
  Serial.print(water_temp_2);
  Serial.print("\t");

  

  ///////////////
  // EC sensor //
  ///////////////
  int TDS_raw;
  //int TEMP_raw for waterproof LM35;
  float Voltage;
  float TDS_25;
  float EC_25;
  float av_EC;
  float EC;

  float sum = 0;
  float t = 25; // VALUE FROM SENSOR!!!
  
  for (int i=0 ; i<5; i++){
    TDS_raw = analogRead(tds_sensor_pin);
    Voltage = TDS_raw*5/1024.0;       //Convert analog reading to Voltage
    TDS_25=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value (original)
    EC_25 = TDS_25*2;
    EC = (1 + a*(t - 25))*EC_25;
    sum = sum + EC;        //sum formula for the following average calculation
    delay(10);
  }

  av_EC = sum / 5;  // average of 5 samples
  Serial.print("EC (uS): "); 
  Serial.println(av_EC);

  
  void loop(){
    tone(buzzer_pin, 400); //4000 in real life
    delay(500);

    noTone(buzzer_pin);
    delay(500);
  }
}

// REFERENCES:
// 1. Code for 1-Wire protocol: https://lastminuteengineers.com/multiple-ds18b20-arduino-tutorial/
// 2. DS18B20's datasheet : https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
// 3. Official code for pH probe and its description: https://wiki.dfrobot.com/PH_meter_SKU__SEN0161_
  // 4. Original code for Grove TDS probe and its description: https://wiki.seeedstudio.com/Grove-TDS-Sensor/
// 5. Temperature compensation for EC: https://www.aqion.de/site/112Source: https://www.aqion.de/site/112

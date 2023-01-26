////////////////////////////////////////////////////////////////////
//////////////////////   REQUIRED LIBRARIES   //////////////////////
////////////////////////////////////////////////////////////////////
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

////////////////////////////////////////////////////////////////////
/////////////////////////   PINOUT MAP   /////////////////////////// 
////////////////////////////////////////////////////////////////////

////////// ANALOG //////////
const byte ph_sensor_pin = A0;           // Analog 0    X1
const byte tds_sensor_pin = A1;          // Analog 1    X1

////////// DIGITAL //////////
const byte water_temp_pin = 2;           // Digital 0   X1
const byte led_strip_1_pin = 3;          // Digital 1   X1
const byte led_strip_2_pin = 4;          // Digital 2   X1
const byte water_level_trig_pin = 5;     // Digital 3   X1
const byte water_level_echo_pin = 6;     // Digital 4   X1
const byte buzzer_pin = 7;               // Digital 5   X1

////////// INTERFACES //////////
const byte raspi_tx_pin = 18;            // UART TX     X1
const byte raspi_rx_pin = 19;            // UART RX     X1
const byte orp_sda_pin = 20;             // I2C SDA     X1
const byte orp_scl_pin = 21;             // I2C SCL     X1

////////// RELAYS //////////
const byte water_rack_pump_1_pin = 22;   // Relay 0     R0
const byte water_rack_pump_2_pin = 23;   // Relay 1     R1
const byte water_heater_pin = 24;        // Relay 2     R2

////////////////////////////////////////////////////////////////////
///////////////////////// MILLIS VARIABLES /////////////////////////
////////////////////////////////////////////////////////////////////

unsigned long startWaterLevelMillis = millis();
unsigned long startBuzzerAlarmMillis = millis();
unsigned long startWaterTemperatureMillis = millis();
unsigned long start_ph_millis = millis();
unsigned long start_ec_millis = millis();

unsigned long currentMillis;

const unsigned long waterLevelPeriod = 1000;
const unsigned long buzzerAlarmPeriod = 1000;
const unsigned long waterTemperaturePeriod = 1000;
const unsigned long ph_period = 1000;
const unsigned long ec_period = 1000;

////////////////////////////////////////////////////////////////////
/////////////////////   SENSOR CONFIGURATIONS   ////////////////////
////////////////////////////////////////////////////////////////////

////////// WATER LEVEL //////////
long duration;
int distance;
int av_dist;
int water_level;

////////// WATER TEMPERATURE //////////
float water_temperature;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(water_temp_pin);
// Pass our oneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);
// Insert address (to get, see DS18B20_get_address):
uint8_t water_temp_address[8] = { 0x28, 0x7F, 0x17, 0xAC, 0x13, 0x19, 0x01, 0x9A };

//////////////////// PH SENSOR ///////////////////////
const float Offset = 0.4;  //deviation compensate
#define LED 10
const int ArrayLength = 40;  //times of collection
int pHArray[ArrayLength];  //Store the average value of the sensor feedback
int pHArrayIndex=0;

/////////////// ELECTRIC CONDUCTIVITY ////////////////
// No calibration required for EC. The electrical conductivity of an aqueous
// solution increases with temperature significantly: about 2 per Celsius
const float a = 0.020;
//int TEMP_raw for waterproof LM35;
float t = 25;
float av_EC;

////////////////////////////////////////////////////////////////////
//////////////////////////   SET-UP LOOP   /////////////////////////
////////////////////////////////////////////////////////////////////

void setup(void){
  pinMode(water_rack_pump_1_pin, OUTPUT);
  pinMode(water_rack_pump_2_pin, OUTPUT);
  pinMode(water_heater_pin, OUTPUT);
  pinMode(water_level_trig_pin, OUTPUT);
  pinMode(water_level_echo_pin, INPUT);
  Serial.begin(9600);
  sensors.begin(); // Start up the library
  
  pinMode(LED,OUTPUT);
}


////////////////////////////////////////////////////////////////////
//////////////////////////   MAIN LOOP   ///////////////////////////
////////////////////////////////////////////////////////////////////
void loop(void){
  currentMillis = millis();
  waterLevel();
  controlWaterLevel();
  waterTemperature();
  controlHeater();
  phLevel();
  ecLevel();
}


void waterLevel(void){
  if (currentMillis - startWaterLevelMillis >= waterLevelPeriod){  
    // 200 ms
    ////////// WATER LEVEL //////////
    float water_level_sum = 0; // sum declaration
    for (int i=0 ; i<5 ; i++){ // 5 samples are taken
      digitalWrite(water_level_trig_pin, LOW); // Clears the water_level_trig_pin condition first
      delayMicroseconds(2);
      digitalWrite(water_level_trig_pin, HIGH); // Sets the water_level_trig_pin HIGH (ACTIVE) for 10 microseconds (time for 8 cycle sonic bursts)
      delayMicroseconds(10); 
      digitalWrite(water_level_trig_pin, LOW);
      duration = pulseIn(water_level_echo_pin, HIGH); // Reads the water_level_echo_pin, returns the sound wave travel time in microseconds
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
      water_level_sum = water_level_sum + distance; // Sum calculation
      delay(20);
    }
    av_dist = round(water_level_sum / 5.0); // one average value of distance in cm
    water_level = map(av_dist, 2, 27, 100, 0); // one average value of distance in % | sensor's range starts from 2 cm (fixed)
    Serial.print("\nDistance: "); // prints average of 5 samples in cm
    Serial.print(av_dist);
    Serial.print(" cm \n");
    Serial.print("\nDistance in %: "); // prints average of 5 samples in %
    Serial.print(water_level);
    Serial.print(" % \n");
    startWaterLevelMillis = currentMillis;
  }
}

void buzzerAlarm(void){
  if(currentMillis - startBuzzerAlarmMillis < buzzerAlarmPeriod){
      noTone(buzzer_pin);
  }
  else if ((currentMillis - startBuzzerAlarmMillis >= buzzerAlarmPeriod) && (currentMillis - startBuzzerAlarmMillis <= (buzzerAlarmPeriod * 2))){
      tone(buzzer_pin, 40); //4000 in real life;
  }
  else{
    startBuzzerAlarmMillis = currentMillis;
    }
}


void controlWaterLevel(void){
    if (water_level <= 20 || water_level >= 90){
    digitalWrite(water_rack_pump_1_pin, LOW);
    digitalWrite(water_rack_pump_2_pin, LOW);
    digitalWrite(water_heater_pin, LOW);
    buzzerAlarm(); // call method for pulsing alarm sound
  }
  else{
    digitalWrite(water_rack_pump_1_pin, HIGH);
    digitalWrite(water_rack_pump_2_pin, HIGH);  
    noTone(buzzer_pin);
  }
}
 
////////// WATER TEMPERATURE SENSOR //////////
 void waterTemperature(void){
  if(currentMillis - startWaterTemperatureMillis >= waterTemperaturePeriod){
    sensors.requestTemperatures();
    Serial.print("Water temp (°C): ");
    printTemperature(water_temp_address);
    Serial.println();
    startWaterTemperatureMillis = currentMillis;
  }
 }
 
 void printTemperature(DeviceAddress deviceAddress){
  water_temperature = sensors.getTempC(deviceAddress);
  Serial.print(water_temperature);
  Serial.print("\t");
}

void controlHeater(void){
    if (water_temperature >= 24){
    digitalWrite(water_heater_pin, LOW);
  }
  else{
    digitalWrite(water_heater_pin, HIGH);
  }
}



void phLevel (void){
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,pHvoltage;
  if (currentMillis - start_ph_millis >  ph_period) //(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex ++] = analogRead(ph_sensor_pin);
      if(pHArrayIndex == ArrayLength){
        pHArrayIndex = 0;
      }
      pHvoltage = avergearray(pHArray, ArrayLength) *5.0 / 1024;
      pHValue = 3.5*pHvoltage+Offset;
      Serial.print("pH Voltage:");
      Serial.print(pHvoltage,2);
      Serial.print("    pH value: ");
      Serial.println(pHValue,2);
      digitalWrite(LED,digitalRead(LED)^1);
      start_ph_millis = currentMillis;
  }
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
  
/////////// ELECTRIC CONDUCTIVITY //////////
void ecLevel (void){
  if (currentMillis - start_ec_millis >= ec_period){
    int TDS_raw;
    float voltage_EC;
    float TDS_25;
    float EC_25;
    float EC_sum = 0;
    float EC;
    for (int i=0 ; i<5; i++){
      TDS_raw = analogRead(tds_sensor_pin);
      voltage_EC = TDS_raw*5/1024.0; //Convert analog reading to Voltage
      TDS_25=(133.42/voltage_EC*voltage_EC*voltage_EC - 255.86*voltage_EC*voltage_EC + 857.39*voltage_EC)*0.5; //Convert voltage value to TDS value (original)
      EC_25 = TDS_25*2;
      EC = (1 + a*(t - 25))*EC_25;
      EC_sum = EC_sum + EC; //sum formula for the following average calculation
      delay(10);
    }
    av_EC = EC_sum / 5; // average of 5 samples
    Serial.print("EC (uS): "); 
    Serial.println(av_EC);
    start_ec_millis = currentMillis;
  }
}

////////////////////////////////////////////////////////////////////
/////////////////////////   REFERENCES   /////////////////////////// 
////////////////////////////////////////////////////////////////////
// 1. Code for 1-Wire protocol: https://lastminuteengineers.com/multiple-ds18b20-arduino-tutorial/
// 2. DS18B20's datasheet : https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
// 3. Official code for pH probe and its description: https://wiki.dfrobot.com/PH_meter_SKU__SEN0161_
// 4. Original code for Grove TDS probe and its description: https://wiki.seeedstudio.com/Grove-TDS-Sensor/
// 5. Temperature compensation for EC: https://www.aqion.de/site/112Source: https://www.aqion.de/site/112

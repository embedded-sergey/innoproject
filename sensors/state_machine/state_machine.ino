////////////////////////////////////////////////////////////////////
//////////////////////   REQUIRED LIBRARIES   //////////////////////
////////////////////////////////////////////////////////////////////
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "ModbusRtu.h" 

// Forward declaration of all functions
void stateMachine();
void displayState(String currentState);

////////////////////////////////////////////////////////////////////
//////////////////////////   PINOUT MAP   ////////////////////////// 
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
const byte raspi_serial = 1;             // UART1       USB
const byte orp_sensor = 2;               // UART2       X1
const byte modbus_serial = 3;            // UART3       RS485
const byte orp_sda_pin = 20;             // I2C SDA     X1
const byte orp_scl_pin = 21;             // I2C SCL     X1

////////// RELAYS //////////
const byte water_rack_pump_1_pin = 22;   // Relay 0     R0
const byte water_rack_pump_2_pin = 23;   // Relay 1     R1
const byte water_heater_pin = 24;        // Relay 2     R2



////////////////////////////////////////////////////////////////////
///////////////////////   MILLIS VARIABLES   ///////////////////////
////////////////////////////////////////////////////////////////////

unsigned long start_co2_millis = millis();
unsigned long start_water_level_millis = millis();
unsigned long start_buzzer_alarm_millis = millis();
unsigned long start_water_temperature_millis = millis();
unsigned long start_ph_millis = millis();
unsigned long start_ec_millis = millis();
unsigned long start_print_all_millis = millis();

unsigned long currentMillis;

const unsigned long co2_period = 0;
const unsigned long water_level_period = 6000;
const unsigned long buzzer_alarm_period = 6500;
const unsigned long water_temperature_period = 7000;
const unsigned long ph_period = 8000;
const unsigned long ec_period = 9000;
const unsigned long print_all_period = 10000;



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
const byte LED = 10;
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
/////////////////////   MODBUS CONFIGURATIONS   ////////////////////
////////////////////////////////////////////////////////////////////

const byte MasterModbusAdd = 0;  // Master is always 0, slaves: from 1 to 247
const byte SlaveModbusAdd_GMP252_1 = 239; // Vaisala probes GMP252 for test
const byte SlaveModbusAdd_GMP252_2 = 240; //  environments 1 & 2, respectively

Modbus ControllinoModbusMaster(MasterModbusAdd, modbus_serial, 0);  
uint16_t ModbusSlaveRegisters[8];
modbus_t ModbusQuery[2]; // the number of queries to slave device(s)

uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query
unsigned long WaitingTime;
float GMP252_1_CO2;
float GMP252_2_CO2;



////////////////////////////////////////////////////////////////////
//////////////////////////   SET-UP LOOP   /////////////////////////
////////////////////////////////////////////////////////////////////

void setup(void){
  pinMode(water_rack_pump_1_pin, OUTPUT);
  pinMode(water_rack_pump_2_pin, OUTPUT);
  pinMode(water_heater_pin, OUTPUT);
  pinMode(water_level_trig_pin, OUTPUT);
  pinMode(water_level_echo_pin, INPUT);
  pinMode(LED,OUTPUT);
  
  Serial.begin(19200);
  sensors.begin(); // Start up the library
  ControllinoModbusMaster.begin(19200, SERIAL_8N2); // 
  ControllinoModbusMaster.setTimeOut( 1000 ); // if there is no answer in 5000 ms, roll over

  // ModbusQuery 0: read registers from GMP252_1
  ModbusQuery[0].u8id = SlaveModbusAdd_GMP252_1; // slave address
  ModbusQuery[0].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0; // start address in slave
  ModbusQuery[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusSlaveRegisters; // pointer to a memory array in the CONTROLLINO

  // ModbusQuery 1: read registers from GMP252_2
  ModbusQuery[1].u8id = SlaveModbusAdd_GMP252_2; // slave address
  ModbusQuery[1].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[1].u16RegAdd = 0; // start address in slave
  ModbusQuery[1].u16CoilsNo = 4; // number of elements (coils or registers) to read
  ModbusQuery[1].au16reg = ModbusSlaveRegisters+4; // pointer to a memory array in the CONTROLLINO

  // Serial configurations of ModbusRTU: baud-rate, data bits, parity, stop bits 
  ControllinoModbusMaster.begin(19200, SERIAL_8N2); // 
  ControllinoModbusMaster.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
 
  WaitingTime = millis();
  myState = 0;
  currentQuery = 0; 
}



////////////////////////////////////////////////////////////////////
//////////////////////////   MAIN LOOP   ///////////////////////////
////////////////////////////////////////////////////////////////////

void loop(void){
  stateMachine();
  }




  
/* 
  co2Level();
  waterLevel();
  controlWaterLevel();
  waterTemperature();
  controlHeater();
  phLevel();
  ecLevel();
  printAll();



unsigned long start_co2_millis = millis();
unsigned long start_water_level_millis = millis();
unsigned long start_buzzer_alarm_millis = millis();
unsigned long start_water_temperature_millis = millis();
unsigned long start_ph_millis = millis();
unsigned long start_ec_millis = millis();
unsigned long start_print_all_millis = millis();

unsigned long currentMillis;

const unsigned long co2_period = 0;
const unsigned long water_level_period = 6000;
const unsigned long buzzer_alarm_period = 6500;
const unsigned long water_temperature_period = 7000;
const unsigned long ph_period = 8000;
const unsigned long ec_period = 9000;
const unsigned long print_all_period = 10000;
*/


void waterLevel(void){
    // 200 ms
    float water_level_sum = 0; // sum declaration
    for (int i=0 ; i<5 ; i++){ // 5 samples are taken
      digitalWrite(water_level_trig_pin, LOW); // Clears the water_level_trig_pin condition first
      delayMicroseconds(2);
      digitalWrite(water_level_trig_pin, HIGH); // Sets the water_level_trig_pin HIGH (ACTIVE) for 10 microseconds (time for 8 cycle sonic bursts)
      delayMicroseconds(10); 
      digitalWrite(water_level_trig_pin, LOW);
      duration = pulseIn(water_level_echo_pin, HIGH, 200); // Reads the water_level_echo_pin, returns the sound wave travel time in microseconds
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
      water_level_sum = water_level_sum + distance; // Sum calculation
      delay(20);
    }
    av_dist = round(water_level_sum / 5.0); // one average value of distance in cm
    water_level = map(av_dist, 2, 27, 100, 0); // one average value of distance in % | sensor's range starts from 2 cm (fixed)
/*    Serial.print("Distance in cm: "); // prints average of 5 samples in cm
    Serial.print(av_dist);
    Serial.println("\t Distance in %: "); // prints average of 5 samples in %
    */
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

void buzzerAlarm(void){
      tone(buzzer_pin, 40); //4000 in real life;
}



void stateMachine() {
    static unsigned long start_machine = millis();
    const unsigned long idle_period = 1000;
    const unsigned long water_level_period = 2000;
    const unsigned long emergency_period = 3000;

    // Timer for notification process has completed
    static unsigned long beepMillis;

    // Declare the states in meaningful English. Enums start enumerating
    // at zero, incrementing in steps of 1 unless overridden. We use an
    // enum 'class' here for type safety and code readability
    enum class controllinoState : uint8_t {
        IDLE,  // defaults to 0
        WATERLEVEL,//RESET    // defaults to 1
        EMERGENCY,   // defaults to 2
    };

    // Keep track of the current State (it's an controllinoState variable)
    static controllinoState currentState = controllinoState::IDLE;

    // Process according to our State Diagram
    switch (currentState) {
        case controllinoState::IDLE:
            // idling for 1000ms
            if (millis() - start_machine >= idle_period) {
                displayState("IDLE state");
                currentState = controllinoState::WATERLEVEL;
            }
            break;

        // Someone pushed the 'call elevator' button - an input
        case controllinoState::WATERLEVEL:

           
            if (millis() - start_machine >= water_level_period) {
                displayState("WATERLEVEL state");
                waterLevel();
                // Move to next state
                currentState = controllinoState::EMERGENCY;
            }
            break;

        // Elevator has EMERGENCY
        case controllinoState::EMERGENCY:


            if (millis() - start_machine >= (emergency_period)) {
                displayState("EMERGENCY State");
                controlWaterLevel();
                // Move to next state
                currentState = controllinoState::IDLE;
                start_machine = millis();
            }
            
            break;

        default:
            // Nothing to do here
            Serial.println("'Default' Switch Case reached - Error");
    }
}

void displayState(String currentState){
    static String prevState = "";

    if (currentState != prevState) {
        Serial.println(currentState);
        prevState = currentState;
    }
}


/*


void co2Level(void){
  if (currentMillis - start_co2_millis >= co2_period){  
    switch( myState ) {
      case 0: 
      if (millis() > WaitingTime) myState++; // wait state
      break;
      
      case 1: 
      ControllinoModbusMaster.query( ModbusQuery[currentQuery] ); // send query (only once)
      myState++;
      currentQuery++;
      if (currentQuery == 2) {
        currentQuery = 0;
      }
      break;
      
      case 2:
      ControllinoModbusMaster.poll(); // check incoming messages
      if (ControllinoModbusMaster.getState() == COM_IDLE){   // response from the slave was received
        myState = 0;
        WaitingTime = millis() + 500; 
          
        if (currentQuery == 0){
          unsigned long *GMP252_1_CO2_uint32;
          GMP252_1_CO2_uint32 = (unsigned long*)&GMP252_1_CO2;
          *GMP252_1_CO2_uint32 = (unsigned long)ModbusSlaveRegisters[1]<<16 | ModbusSlaveRegisters[0]; // Float - Mid-Little Endian CDAB
        }
           
        if (currentQuery == 1){
          unsigned long *GMP252_2_CO2_uint32;
          GMP252_2_CO2_uint32 = (unsigned long*)&GMP252_2_CO2;
          *GMP252_2_CO2_uint32 = (unsigned long)ModbusSlaveRegisters[5]<<16 | ModbusSlaveRegisters[4]; // Float - Mid-Little Endian CDAB
        }
        
        Serial.print("CO2_ppm_env_1: ");  
        Serial.print(GMP252_1_CO2, 2);
        Serial.print(";\t");
        Serial.print("CO2_ppm_env_2: ");
        Serial.println(GMP252_2_CO2, 2);
      }
      break;
    }
  }
}

void waterLevel(void){
  if (currentMillis - start_water_level_millis >= water_level_period){  
    // 200 ms
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

void buzzerAlarm(void){
  if(currentMillis - start_buzzer_alarm_millis < buzzer_alarm_period){
      noTone(buzzer_pin);
  }
  else if ((currentMillis - start_buzzer_alarm_millis >= buzzer_alarm_period) && (currentMillis - start_buzzer_alarm_millis <= (buzzer_alarm_period * 2))){
      tone(buzzer_pin, 40); //4000 in real life;
  }
  else{
    }
}

void waterTemperature(void){
  if(currentMillis - start_water_temperature_millis >= water_temperature_period){
    sensors.requestTemperatures();
    Serial.print("Water temp (°C): ");
    printTemperature(water_temp_address);
    Serial.println();
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
//  digitalWrite(water_heater_pin, HIGH);
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
  }
}


void printAll (void){
  if (currentMillis - start_print_all_millis >= print_all_period){
    Serial.print("ALL: ");
    Serial.print(GMP252_1_CO2, 2);
    Serial.print(";\t");
    Serial.println(GMP252_2_CO2, 2);
    
    start_co2_millis = currentMillis;
    start_water_level_millis = currentMillis;
    start_buzzer_alarm_millis = currentMillis;
    start_water_temperature_millis = currentMillis;
    start_ph_millis = currentMillis;
    start_ec_millis = currentMillis;
    start_print_all_millis = currentMillis;
    
    Serial.print("STOP: ");
    Serial.println(currentMillis);
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
*/

////////////////////////////////////////////////////////////////////
//////////////////////   REQUIRED LIBRARIES   //////////////////////
////////////////////////////////////////////////////////////////////
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h> 
#include "ModbusRtu.h" 

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
//const byte orp_serial = 1;               // UART1      X1
const byte raspi_serial = 2;             // UART2       X1
const byte modbus_serial = 3;            // UART3       RS485

////////// RELAYS //////////
const byte water_heater_pin = 26;        // Relay 2     R2
const byte water_pump_pin = 25;   // Relay 3     R3
const byte fan_pin = 26;   // Relay 4     R4

SoftwareSerial mySerial(53, 20);
////////////////////////////////////////////////////////////////////
/////////////////////   SENSOR CONFIGURATIONS   ////////////////////
////////////////////////////////////////////////////////////////////

////////// WATER LEVEL //////////
long duration;
int distance;
int av_dist;
int water_level;

////////// WATER TEMPERATURE //////////
float av_temp;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(water_temp_pin);
// Pass our oneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);
// Insert address (to get, see DS18B20_get_address):
uint8_t sensor1[8] = { 0x28, 0x7F, 0x17, 0xAC, 0x13, 0x19, 0x01, 0x9A };
uint8_t sensor2[8] = { 0x28, 0xFF, 0xC8, 0xC2, 0xC1, 0x16, 0x04, 0xB5 };

//////////////////// PH SENSOR ///////////////////////
const float Offset = 0.4;  //deviation compensate
const byte LED = 10;
float av_ph;

/////////////// ELECTRIC CONDUCTIVITY ////////////////
// No calibration required for EC. The electrical conductivity of an aqueous
// solution increases with temperature significantly: about 2 per Celsius
const float a = 0.020;
//int TEMP_raw for waterproof LM35;
float t = 25;
float av_EC;

//////////////////////// ORP /////////////////////////
String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
float ORP;  



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
  pinMode(water_pump_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(water_heater_pin, OUTPUT);
  pinMode(water_level_trig_pin, OUTPUT);
  pinMode(water_level_echo_pin, INPUT);
  pinMode(LED,OUTPUT);
  
  Serial.begin(19200);
  mySerial.begin(19200);  
  sensors.begin(); // Start up the library
  inputstring.reserve(10);                            
  sensorstring.reserve(30);

  ControllinoModbusMaster.begin(19200, SERIAL_8N2); // 
  ControllinoModbusMaster.setTimeOut(1000); // if there is no answer in 5000 ms, roll over

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
  ControllinoModbusMaster.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
 
  WaitingTime = millis() + 1000;
  myState = 0;
  currentQuery = 0; 
}



////////////////////////////////////////////////////////////////////
//////////////////////////   MAIN LOOP   ///////////////////////////
////////////////////////////////////////////////////////////////////



void loop(void){
  stateMachine(); // if not busy, waiting for triggers/events.
  }


void stateMachine() {
    static unsigned long start_machine = millis();
    // first 1000 ms are reserved for the last state processing
    static unsigned long start_idle = 2000;
    static unsigned long start_water_level = 4000; 
    static unsigned long start_emergency = 6000;
    static unsigned long start_water_temperature = 8000;
    static unsigned long start_ec = 10000;
    static unsigned long start_ph = 12000;
    static unsigned long start_orp = 14000;
    static unsigned long start_co2 = 16000;
    
    
    /* PREVIOUS CODE
    static unsigned long start_co2 = 2000; // 1000 ms per probe
    static unsigned long start_idle = 1000;
    static unsigned long start_fans = 4000; 
    static unsigned long start_orp = 5000;
    static unsigned long start_ec = 6000;
    static unsigned long start_ec_pumps = 7000;
    static unsigned long start_ph = 8000;
    static unsigned long start_ph_pumps = 9000;
    static unsigned long start_water_temperature = 10000;
    static unsigned long start_heater = 11000;
    static unsigned long start_water_level = 12000;  
    static unsigned long start_emergency = 13000;
    static unsigned long start_sd_rtc = 14000;
    static unsigned long start_to_raspi = 15000;*/
    
    // Declare the states in meaningful English. Enums start enumerating
    // at zero, incrementing in steps of 1 unless overridden. We use an
    // enum 'class' here for type safety and code readability
    enum class controllinoState : uint8_t {
        IDLE,  // defaults to 0
        WATER,
        EMERGENCY,   // defaults to 3
        TEMP,
        EC,
        PH,
        ORP,         //needs adjustment
        CO2,       //needs adjustment
    };

    // Keep track of the current State (it's an controllinoState variable)
    static controllinoState currentState = controllinoState::IDLE;

    // Process according to our State Diagram
    switch (currentState) {
        case controllinoState::IDLE:  
            // idling for 1000ms     
            if (millis() - start_machine >= start_idle) {
                displayState("IDLE state");
                currentState = controllinoState::WATER;
            }
            break;

        case controllinoState::WATER:
          if (millis() - start_machine >= start_water_level) {
            displayState("Water level ");
            waterLevel();
            currentState = controllinoState::EMERGENCY;
          }
          break;

        case controllinoState::EMERGENCY:
            if (millis() - start_machine >= (start_emergency)) {
                displayState("EMERGENCY State");
                  controlWaterLevel();
                currentState = controllinoState::TEMP;
            }
            break;

        case controllinoState::TEMP:
            if (millis() - start_machine >= (start_water_temperature)) {
                displayState("Water tC° State");
                waterTemperature();
                currentState = controllinoState::EC;
            }
            break;           

        case controllinoState::EC:
          if (millis() - start_machine >= start_ec) {
            displayState("EC State ");
            ecLevel();
            currentState = controllinoState::PH;
          }
          break;

        case controllinoState::PH:
          if(millis() - start_machine >= start_ph){
            displayState("PH State");
            phLevel();
            currentState = controllinoState::ORP;
          }
          break;

        case controllinoState::ORP:
          if(millis() - start_machine >= start_orp){
            displayState("ORP State");
            serialEvent();
            serialEvent1();
            orp();
            currentState = controllinoState::CO2;
          }
          break;

        case controllinoState::CO2:
          if(millis() - start_machine >= start_co2){
            displayState("CO2 State");
            co2Level();
            
            // Move to next state
            currentState = controllinoState::IDLE;

            start_machine = millis(); // Comment this line to remove scheduling 
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



//////////////////////////////////////////////////////////////////
//////////////////////////  FUNCTIONS   //////////////////////////
//////////////////////////////////////////////////////////////////



void waterLevel(void){
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
    }
    av_dist = round(water_level_sum / 5.0); // one average value of distance in cm
    water_level = map(av_dist, 2, 22, 100, 0); // one average value of distance in % | sensor's range starts from 2 cm (fixed)
    Serial.print("  Distance in cm: "); // prints average of 5 samples in cm
    Serial.print(av_dist);
    Serial.print("\t Distance in %: "); // prints average of 5 samples in %
    Serial.println(water_level); 
}



void controlWaterLevel(void){
  if (water_level <= 20 || water_level >= 90){
    digitalWrite(water_pump_pin, LOW);
    digitalWrite(fan_pin, LOW);
    digitalWrite(water_heater_pin, LOW);
    buzzerAlarm(); // call method for pulsing alarm sound
  }
  else{
    digitalWrite(water_pump_pin, HIGH);
    digitalWrite(fan_pin, HIGH);  
    noTone(buzzer_pin);
  }
}



void buzzerAlarm(void){
      tone(buzzer_pin, 40); //4000 in real life;
}



void ecLevel (void){
      int TDS_raw;
      float voltage_EC;
      float TDS_25;
      float EC_25;
      float EC_sum = 0;
      float EC;
      for (int i=0 ; i<5; i++){
        TDS_raw = analogRead(tds_sensor_pin);
        voltage_EC = TDS_raw*5/1024.0; //Convert analog reading to Voltage
        TDS_25=(133.42*voltage_EC*voltage_EC*voltage_EC - 255.86*voltage_EC*voltage_EC + 857.39*voltage_EC)*0.5; //Convert voltage value to TDS value (original)
        EC_25 = TDS_25*2;
        EC = (1 + a*(av_temp - 25))*EC_25;    //real temp compensation 
        EC_sum = EC_sum + EC; //sum formula for the following average calculation
        delay(10);
      }
      av_EC = EC_sum / 5; // average of 5 samples
      Serial.print("  EC (uS): "); 
      Serial.println(av_EC);
}



void phLevel (void){
  float ph_value, voltage, ph_raw, ph_sum = 0;
  
  for (int k=0; k<5; k++){
    ph_raw = analogRead(ph_sensor_pin);
    voltage = ph_raw*5/1024.0;
    ph_value = 3.5*voltage+Offset;
    ph_sum = ph_sum + ph_value;
  }
  av_ph = ph_sum / 5;
  Serial.print("    pH value: ");
  Serial.println(av_ph);
}



void waterTemperature(void){
  sensors.requestTemperatures();
  float tempC1 = sensors.getTempC(sensor1);
  Serial.print("Temp1 (°C): " + (String)tempC1 + "\t");
  
  float tempC2 = sensors.getTempC(sensor2);
  Serial.print("Temp2 (°C): " + (String)tempC2 + "\n");

  av_temp = (tempC1 + tempC2)/2;
  Serial.println(av_temp);
}


void orp() {                                         //here we go...
  Serial.println(input_string_complete);
  if (input_string_complete == true) {                //if a string from the PC has been received in its entirety
    mySerial.print(inputstring);                       //send that string to the Atlas Scientific product
    mySerial.print('\r');                              //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }
  Serial.println(sensor_string_complete);
  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
  }
  sensorstring = "";                                  //clear the string:
  sensor_string_complete = false;                     //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
}
void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}
void serialEvent1() {                                 //if the hardware serial port_3 receives a char
  sensorstring = mySerial.readStringUntil(13);         //read the string until we see a <CR>
  sensor_string_complete = true;                      //set the flag used to tell if we have received a completed string from the PC
}


void co2Level(void){
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
      WaitingTime = millis() + 1000; 
        
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



/*
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



void controlHeater(void){
    if (water_temperature >= 24){
    digitalWrite(water_heater_pin, LOW);
  }
  else{
//  digitalWrite(water_heater_pin, HIGH);
  }
}
*/
/*

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

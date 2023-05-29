////////////////////////////////////////////////////////////////////
//////////////////////   REQUIRED LIBRARIES   //////////////////////
////////////////////////////////////////////////////////////////////
#include <Controllino.h>
#include <OneWire.h>
#include <DallasTemperature.h> 
#include <FastLED.h>
#include "ModbusRtu.h" 
////////////////////////////////////////////////////////////////////
//////////////////////////   PINOUT MAP   ////////////////////////// 
////////////////////////////////////////////////////////////////////

////////// ANALOG //////////
const byte ph_sensor_pin = A0;           // Analog 0    X1
const byte tds_sensor_pin = A1;          // Analog 1    X1

////////// DIGITAL //////////
const byte water_temp_pin = 2;           // Digital 0   X1
const byte led_strip_pin = 3;          // Digital 1   X1
const byte water_level_trig_pin = 5;     // Digital 3   X1
const byte water_level_echo_pin = 6;     // Digital 4   X1
const byte buzzer_pin = 7;               // Digital 5   X1

////////// INTERFACES //////////
SoftwareSerial mySerial(53, 20);         // SoftUART    X1
const byte raspi_serial = 1;             // UART1       X1
const byte modbus_serial = 3;            // UART3       RS485

////////// RELAYS //////////
const byte fan_pin = 25;                 // Relay 3     R3
const byte water_pump_pin = 26;          // Relay 4     R4



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

///////////////////// LED STRIP //////////////////////
const byte NUM_LEDS = 30;
CRGB leds[NUM_LEDS];

///////////////////// TEMPERATURE ////////////////////
float tempC1;
float tempC2;

///////////////////// RASPBERRY //////////////////////
String fromPi;
byte alarm_flag;

/////////////   MODBUS CONFIGURATIONS   //////////////
const byte MasterModbusAdd = 0;  // Master is always 0, slaves: from 1 to 247
const byte SlaveModbusAdd_GMP252 = 239; // Vaisala probes GMP252 for test
const byte SlaveModbusAdd_HPP271 = 240; //  environments 1 & 2, respectively
float GMP252_CO2;
float HPP271_H2O2;
Modbus ControllinoModbusMaster(MasterModbusAdd, modbus_serial, 0);  
uint16_t ModbusSlaveRegisters[8];
modbus_t ModbusQuery[2]; // the number of queries to slave device(s)
uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query
unsigned long WaitingTime;



////////////////////////////////////////////////////////////////////
//////////////////////////   SET-UP LOOP   /////////////////////////
////////////////////////////////////////////////////////////////////

void setup(void){
  pinMode(water_pump_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(water_level_trig_pin, OUTPUT);
  pinMode(water_level_echo_pin, INPUT);
  pinMode(LED,OUTPUT);
  
  Serial.begin(19200);
  Serial1.begin(19200);
  mySerial.begin(19200);  
  sensors.begin(); // Start up the library
  inputstring.reserve(10);                            
  sensorstring.reserve(30);
  FastLED.addLeds<WS2812, led_strip_pin, GRB>(leds, NUM_LEDS);
  
  ControllinoModbusMaster.begin(19200, SERIAL_8N2); // 
  ControllinoModbusMaster.setTimeOut(1000); // if there is no answer in 5000 ms, roll over

  // ModbusQuery 0: read registers from GMP252
  ModbusQuery[0].u8id = SlaveModbusAdd_GMP252; // slave address
  ModbusQuery[0].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0; // start address in slave
  ModbusQuery[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusSlaveRegisters; // pointer to a memory array in the CONTROLLINO

  // ModbusQuery 1: read registers from HPP271
  ModbusQuery[1].u8id = SlaveModbusAdd_HPP271; // slave address
  ModbusQuery[1].u8fct = 3; // function code (this one is registers read)
  ModbusQuery[1].u16RegAdd = 0; // start address in slave
  ModbusQuery[1].u16CoilsNo = 4; // number of elements (coils or registers) to read
  ModbusQuery[1].au16reg = ModbusSlaveRegisters+4; // pointer to a memory array in the CONTROLLINO

  // Serial configurations of ModbusRTU: baud-rate, data bits, parity, stop bits 
  ControllinoModbusMaster.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  WaitingTime = millis();
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
    // first 2000 ms are reserved for the last state processing
    static unsigned long start_idle = 2000;
    static unsigned long start_co2 = 3000;
    static unsigned long start_h2o2 = 4000;
    static unsigned long start_fan = 5000;
    static unsigned long start_buzzer = 5300;
    static unsigned long start_orp = 5500;  // at 24V requires minimum 2 sec
    static unsigned long start_ec = 6500;
    static unsigned long start_ph = 7000;
    static unsigned long start_water_temperature = 7500;
    static unsigned long start_led_strip = 8500;
    static unsigned long start_water_level = 9000; 
    static unsigned long start_emergency = 9500;
    static unsigned long start_to_raspi = 10000;
    
    // Declare the states in meaningful English. Enums start enumerating
    // at zero, incrementing in steps of 1 unless overridden. We use an
    // enum 'class' here for type safety and code readability
    enum class controllinoState : uint8_t {
        IDLE,  
        CO2,
        H2O2,
        FAN,
        BUZZER,
        ORP,  
        EC,
        PH,
        TEMP,
        LED,
        WATER,
        EMERGENCY, 
        RASPBERRY,
    };

    // Keep track of the current State (it's an controllinoState variable)
    static controllinoState currentState = controllinoState::IDLE;

    // Process according to our State Diagram
    switch (currentState) {
        case controllinoState::IDLE:  
          if (millis() - start_machine >= start_idle) {
            displayState("IDLE state");
            currentState = controllinoState::CO2;
          }
          break;
            
        case controllinoState::CO2:
          if(millis() - start_machine >= start_co2){
            displayState("CO2 State");
            while(millis() - start_machine <= start_h2o2){
              co2Level();
            }
            currentState = controllinoState::H2O2;
          }
          break;
          
        case controllinoState::H2O2:
          if(millis() - start_machine >= start_h2o2){
            displayState("H2O2 State");
            while(millis() - start_machine <= start_fan){
              h2o2Level();
            }
            currentState = controllinoState::FAN;
          }
          break;
          
        case controllinoState::FAN:
          if (millis() - start_machine >= start_fan) {
            displayState("Fan status");
            controlFan();
            currentState = controllinoState::BUZZER;
          }
          break;

        case controllinoState::BUZZER:
          if (millis() - start_machine >= start_fan) {
            displayState("Buzzer status");
            buzzerOff();
            currentState = controllinoState::ORP;
          }
          break;
                 
        case controllinoState::ORP:
          if(millis() - start_machine >= start_orp){
            displayState("ORP State");
            serialEvent();
            serialEvent1();
            orp();
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
            currentState = controllinoState::TEMP;
          }
          break;

        case controllinoState::TEMP:
          if (millis() - start_machine >= (start_water_temperature)) {
            displayState("Water Temperature State");
            waterTemperature();
            currentState = controllinoState::LED;
          }
          break;  


        case controllinoState::LED:
          if (millis() - start_machine >= (start_led_strip)) {
            displayState("LED State");
            controlLEDstrip();
            currentState = controllinoState::WATER;
          }
          break;            

        case controllinoState::WATER:
          if (millis() - start_machine >= start_water_level) {
            displayState("WATER State");
            waterLevel();
            currentState = controllinoState::EMERGENCY;
          }
          break;

        case controllinoState::EMERGENCY:
          if (millis() - start_machine >= (start_emergency)) {
            displayState("EMERGENCY State");
              controlWaterLevel();
            currentState = controllinoState::RASPBERRY;
          }
          break;
          
        case controllinoState::RASPBERRY:
          if (millis() - start_machine >= (start_to_raspi)) {
            displayState("RASPBERRY State");
              transmitToRasp();
            currentState = controllinoState::IDLE;

            start_machine = millis(); //RESET TIME LINE!
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
      WaitingTime = millis() + 500; 
      
      if (currentQuery == 0){
        // registers write was proceed
        // Serial.println("---------- WRITE RESPONSE RECEIVED ----");
        // Serial.println("");
      }
      
      if (currentQuery == 1){
        unsigned long *GMP252_CO2_uint32;
        GMP252_CO2_uint32 = (unsigned long*)&GMP252_CO2;
        *GMP252_CO2_uint32 = (unsigned long)ModbusSlaveRegisters[1]<<16 | ModbusSlaveRegisters[0]; // Float - Mid-Little Endian CDAB
        Serial.print("  CO2_ppm: ");  
        Serial.println(GMP252_CO2, 2);
      }
    }
    break;
  }
}

void h2o2Level(void){
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
        // registers write was proceed
        // Serial.println("---------- WRITE RESPONSE RECEIVED ----");
        // Serial.println("");
      }
      
      if (currentQuery == 1){
        unsigned long *HPP271_H2O2_uint32;
        HPP271_H2O2_uint32 = (unsigned long*)&HPP271_H2O2;
        *HPP271_H2O2_uint32 = (unsigned long)ModbusSlaveRegisters[5]<<16 | ModbusSlaveRegisters[4]; // Float - Mid-Little Endian CDAB
        Serial.print("  H2O2_ppm: ");
        Serial.println(HPP271_H2O2, 2);
      }
    }
    break;
  }
}

void controlFan(void){
  if (GMP252_CO2 > 1200 || HPP271_H2O2 > 1){
    digitalWrite(fan_pin, HIGH);
    Serial.println("  Fan is ON");
  }
  else{
    digitalWrite(fan_pin, LOW);
    Serial.println("  Fan is OFF");
    }
}

void buzzerOff(void){
  noTone(buzzer_pin);
  Serial.println("  Buzzer is OFF");
}


void orp() {                                         
  if (input_string_complete == true) {                //if a string from the PC has been received in its entirety
    mySerial.print(inputstring);                       //send that string to the Atlas Scientific product
    mySerial.print('\r');                              //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }
  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.print("  ORP (mV): "); 
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
  }
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
  Serial.print("  pH value: ");
  Serial.println(av_ph);
}


void waterTemperature(void){
  sensors.requestTemperatures();
  tempC1 = sensors.getTempC(sensor1);
  Serial.println("  Temp1 (°C): " + (String)tempC1);
  
  tempC2 = sensors.getTempC(sensor2);
  Serial.println("  Temp2 (°C): " + (String)tempC2);

  av_temp = (tempC1 + tempC2)/2;
}


void controlLEDstrip(void) {
  if (av_temp < 30 && (water_level > 20 && water_level < 90)){
    for (int i=0; i<6; i++){
      for (int j=0; j<5; j++){
        if (j==2){
          leds[i*5+2] = CRGB (255, 0, 0); //red
        }
        else if(j==4){
          leds[i*5+4] = CRGB (255, 0, 0); //red
          }
        else{
          leds[i*5+j] = CRGB (35, 0, 255); //blue
        }
      }
    }
    FastLED.show();
    Serial.println("  LED strip is ON");
  }
  else{
    FastLED.clear();
    FastLED.show();
    Serial.println("  LED strip is OFF");
  }
}


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
    Serial.print("\n  Distance in %: "); // prints average of 5 samples in %
    Serial.println(water_level);
}


void controlWaterLevel(void){
  if (water_level <= 20 || water_level >= 90){
    digitalWrite(water_pump_pin, LOW);
    FastLED.clear();
    FastLED.show();
    tone(buzzer_pin, 40); //4000 in real life;; // call method for pulsing alarm sound
    Serial.println("  Buzzer is ON");
    Serial.println("  LED is OFF");
    Serial.println("  Pump is OFF");
    Serial.println("  ALARM: check the water level in the tank!!!");
    alarm_flag = 1;
  }
  else{
    digitalWrite(water_pump_pin, HIGH);
    noTone(buzzer_pin);
    Serial.println("  Pump is ON");
    alarm_flag = 0;
  }
}


String transmitToRasp(void){
  /*if (Serial1.available() > 0){
      if (fromPi != ""){
        fromPi = Serial1.readString();
        Serial1.print("I got:");
        Serial1.println(fromPi);
      }
    }
    */
    String response = (String)tempC1 + "&" + (String)tempC2 + "&" + 
      (String)av_ph + "&" + (String)HPP271_H2O2 + "&" + (String)av_EC + "&" + 
      sensorstring + "&" + (String)GMP252_CO2 + "&" + (String)water_level + "&" + 
      (String)alarm_flag;
    Serial1.println(response);
    Serial.println(response);
    return(response);
}

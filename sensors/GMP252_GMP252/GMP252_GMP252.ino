#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include "ModbusRtu.h" /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

#define MasterModbusAdd 0  // Master is always 0, slaves: from 1 to 247
#define SlaveModbusAdd_GMP252_1 239 // Vaisala probes GMP252 for test
#define SlaveModbusAdd_GMP252_2 240 //  environments 1 & 2, respectively

#define RS485Serial 3 // UART Serial3 for RS485 interface.

Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);  
uint16_t ModbusSlaveRegisters[8];
modbus_t ModbusQuery[2]; // the number of queries to slave device(s)

uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query

unsigned long WaitingTime;

void setup() {
  Serial.begin(9600);

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
 
  WaitingTime = millis() + 1000;
  myState = 0;
  currentQuery = 0; 
}

void loop() {
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
        float GMP252_1_CO2;
        unsigned long *GMP252_1_CO2_uint32;
        GMP252_1_CO2_uint32 = (unsigned long*)&GMP252_1_CO2;
        *GMP252_1_CO2_uint32 = (unsigned long)ModbusSlaveRegisters[1]<<16 | ModbusSlaveRegisters[0]; // Float - Mid-Little Endian CDAB
        Serial.print("CO2 for environment 1 (ppm): ");
        Serial.print(GMP252_1_CO2, 2);
        Serial.print(";\t");
      }
         
      if (currentQuery == 1){
        float GMP252_2_CO2;
        unsigned long *GMP252_2_CO2_uint32;
        GMP252_2_CO2_uint32 = (unsigned long*)&GMP252_2_CO2;
        *GMP252_2_CO2_uint32 = (unsigned long)ModbusSlaveRegisters[5]<<16 | ModbusSlaveRegisters[4]; // Float - Mid-Little Endian CDAB
        Serial.print("CO2 for environment 2 (ppm): ");
        Serial.println(GMP252_2_CO2, 2);
      }
    }
    break;
  }
}

// REFERENCES:
// 1. Wiring: https://www.controllino.com/knowledge-base/rs485-modbusrtu/
// 2. Example code: https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library/blob/master/MAXI/DemoModbusRTUMaster/ModbusRtu.h
// 3. From uint32 to float (IEEE-754): https://forum.arduino.cc/t/combine-two-uint16_t-in-one-float/507834/17 
// 4. Hex converter: https://www.scadacore.com/tools/programming-calculators/online-hex-converter/
// 5. Vaisala GMP252: https://docs.vaisala.com/r/M211060EN-J/en-US/GUID-899F92B4-583C-404B-A4BA-0D47330C6573

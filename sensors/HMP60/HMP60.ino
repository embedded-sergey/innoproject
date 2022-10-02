#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include "ModbusRtu.h" /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

// This MACRO defines Modbus master address.
// For any Modbus slave devices are reserved addresses in the range from 1 to 247.
// Important note only address 0 is reserved for a Modbus master device!

#define MasterModbusAdd 0
#define SlaveModbusAdd 240

// This MACRO defines number of the comport that is used for RS 485 interface.
// For MAXI and MEGA RS485 is reserved UART Serial3.
#define RS485Serial 3

// The object ControllinoModbuSlave of the class Modbus is initialized with three parameters.
// The first parametr specifies the address of the Modbus slave device.
// The second parameter specifies type of the interface used for communication between devices - in this sketch - RS485.
// The third parameter can be any number. During the initialization of the object this parameter has no effect.
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 2);

// This uint16 array specified internal registers in the Modbus slave device.
// Each Modbus device has particular internal registers that are available for the Modbus master.
// In this example sketch internal registers are defined as follows:
// (ModbusSlaveRegisters 0 - 1 read only):
// ModbusSlaveRegisters[0000] - Relative humidity 32-bit float %RH
// ModbusSlaveRegisters[0001] - // - most significant
// ModbusSlaveRegisters[0002] - Temperature 32-bit float %RH
// ModbusSlaveRegisters[0003] - // - most significant
uint16_t ModbusSlaveRegisters[4];

// This is an structure which contains a query to an slave device
modbus_t ModbusQuery[2];

uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query

unsigned long WaitingTime;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
  Serial.println("-----------------------------------------");
  Serial.println("CONTROLLINO Modbus RTU Master Test Sketch");
  Serial.println("-----------------------------------------");
  Serial.println("");
  // ModbusQuery 0: read registers
  ModbusQuery[0].u8id = SlaveModbusAdd; // slave address
  ModbusQuery[0].u8fct = 0x03; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0x0000; // start address in slave
  ModbusQuery[0].u16CoilsNo = 0x0004; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusSlaveRegisters; // pointer to a memory array in the CONTROLLINO 

  ControllinoModbusMaster.begin(19200); // baud-rate at 19200
  ControllinoModbusMaster.setTimeOut(2000); // if there is no answer in 5000 ms, roll over
 
  WaitingTime = millis() + 1000;
  myState = 0;
  currentQuery = 1;
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
     if (currentQuery == 2){
       currentQuery = 1;
       }
     break;
   case 2:
     ControllinoModbusMaster.poll(); // check incoming messages
     if (ControllinoModbusMaster.getState() == COM_IDLE){
     // response from the slave was received
     myState = 0;
     WaitingTime = millis() + 1000; 
     // debug printout
     if (currentQuery == 1) {
       // registers read was proceed
       Serial.println("Read response received:");
       Serial.print("Device ID: ");
       Serial.print(SlaveModbusAdd); 
       Serial.print(": ");
       Serial.print(ModbusSlaveRegisters[0], HEX);
       Serial.print(" ");
       Serial.print(ModbusSlaveRegisters[1], HEX);
       Serial.print(" ");
       Serial.print(ModbusSlaveRegisters[2], HEX);
       Serial.print(" ");
       Serial.print(ModbusSlaveRegisters[3], HEX);
       Serial.println("");
       }
     }
     break;
   }
 }

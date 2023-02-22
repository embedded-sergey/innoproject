#include <SPI.h>
#include <Controllino.h>
#include <SoftwareSerial.h>                           

#define  myserial Serial1                             //define tx and rx on CONTROLLINO MAXI. 
                                                      //in this case TX1 18 pin and RX1 19 pin, hence it's Serial1                                  

String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
float ORP;                                            //used to hold a floating point number that is the ORP



void setup() {                                        //set up the hardware
  Serial.begin(9600);                                 //set baud rate for the hardware serial port_0 to 9600
  myserial.begin(9600);                               //set baud rate for the software serial port to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);  
 
}


void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}


void loop() {                                         //here we go...

  if (input_string_complete == true) {                //if a string from the PC has been received in its entirety
    myserial.print(inputstring);                      //send that string to the Atlas Scientific product
    myserial.print('\r');                             //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }

  if (myserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)myserial.read();              //get the char we just received
    sensorstring += inchar;                           //add the char to the var called sensorstring
    if (inchar == '\r') {                             //if the incoming character is a <CR>
      sensor_string_complete = true;                  //set the flag
    }
  }


  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
    /*                                                //uncomment this section to see how to convert the ORP reading from a string to a float 
    if (isdigit(sensorstring[0])) {                   //if the first character in the string is a digit
      ORP = sensorstring.toFloat();                   //convert the string to a floating point number so it can be evaluated by the Arduino
      if (ORP >= 500.0) {                             //if the ORP is greater than or equal to 500
        Serial.println("high");                       //print "high" this is demonstrating that the Arduino is evaluating the ORP as a number and not as a string
      }
      if (ORP <= 499.9) {                             //if the ORP is less than or equal to 499.9
        Serial.println("low");                        //print "low" this is demonstrating that the Arduino is evaluating the ORP as a number and not as a string
      }
    }
   */
    sensorstring = "";                                //clear the string
    sensor_string_complete = false;                   //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}


//REFERENCES:
//The code is based on the official instructions:
//https://files.atlas-scientific.com/Arduino-Uno-ORP-sample-code.pdf

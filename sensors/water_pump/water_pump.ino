#include <Controllino.h>
#define RELAY_1 23  //Relay num. 1, pin 23 to control a pump

void setup() {
  pinMode(RELAY_1, OUTPUT);  
}

void loop() {
  digitalWrite(RELAY_1, HIGH); //pump ON 
}

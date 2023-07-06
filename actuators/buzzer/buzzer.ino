#include <Controllino.h>

 const byte buzzer = 7;

void setup() {
  Serial.begin(9600);  
  pinMode(buzzer, OUTPUT); 
}

void loop() {

tone(buzzer, 400); //4000 in real life
delay(500);

noTone(buzzer);
delay(500);
}

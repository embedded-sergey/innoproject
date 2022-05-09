#include <Controllino.h>
#define echoPin 4
#define trigPin 5 
long duration;
int distance;

void setup() {

pinMode(trigPin, OUTPUT); 
pinMode(echoPin, INPUT); 
Serial.begin(9600);
Serial.println("Water level sensor");
}
void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000); // ### CHECK ###; 1000 is more stable, than orig 10
  
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.34 / 2; // Speed of sound wave divided by 2 (go and back)
  int distance_proc = map(distance, 20, 270, 0, 100); //air buffer 2 cm
  Serial.print("Distance in %: ");
  Serial.print(distance_proc);
  Serial.print(" % ");
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(980);
}

/* References: 
 *  1. Based on HC-SR04 tutorial: https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6
 */

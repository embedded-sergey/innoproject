#include <Controllino.h>
#define echoPin 4
#define trigPin 5

#define RELAY_0 22      //Relay num. 0, pin 22 to control a pump

long duration;
int distance;

void setup() {
pinMode(RELAY_0, OUTPUT);
pinMode(trigPin, OUTPUT); 
pinMode(echoPin, INPUT); 
Serial.begin(9600);
Serial.println("\nWater level sensor");
}

void loop() {

  int distance_perc;    // distance declaration (in %)
  float sum = 0;        // sum declaration
  int av_dist = 0;      // average distance declaration (in cm)
  
  
  for (int i=0 ; i<5 ; i++){         // 5 samples are taken
    digitalWrite(trigPin, LOW);      // Clears the trigPin condition first
    delayMicroseconds(2);
  
    digitalWrite(trigPin, HIGH);     // Sets the trigPin HIGH (ACTIVE) for 10 microseconds (time for 8 cycle sonic bursts)
    delayMicroseconds(10); 
  
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
    
    distance = duration * 0.034 / 2;    // Speed of sound wave divided by 2 (go and back)
    sum = sum + distance;               // Sum calculation
    delay(20);
  }

  av_dist = round(sum / 5.0);                          // one average value of distance in cm
  distance_perc = map(av_dist, 2, 27, 0, 100);         // one average value of distance in % | sensor's range starts from 2 cm (fixed)
  
  Serial.print("\nDistance: ");          // prints average of 5 samples in cm
  Serial.print(av_dist);
  Serial.print(" cm \n");

  Serial.print("\nDistance in %: ");     // prints average of 5 samples in %
  Serial.print(distance_perc);
  Serial.print(" % \n");

  delay(900);

if (distance_perc <= 50){               //pump starts to work, when the main tank is less or 50% filled
  digitalWrite(RELAY_0, HIGH);
  }
  else if (distance_perc >= 100) {      //tank won't be overfilled
    digitalWrite(RELAY_0, LOW);
  }
  else{
    digitalWrite(RELAY_0, LOW);         //in all other cases pump does not work
  }

}

/* References: 
 *  1. Based on HC-SR04 tutorial: https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6
 */

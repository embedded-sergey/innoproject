#include <Controllino.h>
#define SERIAL Serial
#define sensorPin A0

int sensorValue = 0;

void setup() {
    SERIAL.begin(9600);
}
void loop() {
  float ECValue = 0;
  float Voltage = 0;
  float sum = 0;
  float av_EC = 0;
  
  for (int i=0 ; i<5; i++){
    sensorValue = analogRead(sensorPin);
    Voltage = sensorValue*5/1024.0;       //Convert analog reading to Voltage
    ECValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage);       //Convert voltage value to EC value
    sum = sum + ECValue;        //sum formula for the following average calculation
    delay(1000);
  }

av_EC = sum / 5;        //average of 5 samples
SERIAL.print("EC Value = "); 
SERIAL.print(av_EC);
SERIAL.println(" uS/cm");
}

//Reference: https://wiki.seeedstudio.com/Grove-TDS-Sensor/

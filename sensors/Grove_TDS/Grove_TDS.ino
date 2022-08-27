#include <Controllino.h>
#define SERIAL Serial
#define sensorPin A1

// No calibration required
const float a = 0.020; //The electrical conductivity (EC) of an aqueous solution increases with temperature significantly: about 2 per degree Celsius.
  
void setup() {
    SERIAL.begin(9600);
}
void loop() {
  int TDS_raw;
  //int TEMP_raw for waterproof LM35;
  float Voltage;
  float TDS_25;
  float EC_25;
  float av_EC;
  float EC;

  float sum = 0;
  float t = 25; // VALUE FROM SENSOR!!!
  
  for (int i=0 ; i<5; i++){
    TDS_raw = analogRead(sensorPin);
    Voltage = TDS_raw*5/1024.0;       //Convert analog reading to Voltage
    TDS_25=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value (original)
    EC_25 = TDS_25*2;
    EC = (1 + a*(t - 25))*EC_25;
    sum = sum + EC;        //sum formula for the following average calculation
    delay(200);
  }

av_EC = sum / 5;  // average of 5 samples
SERIAL.print("EC = "); 
SERIAL.print(av_EC);
SERIAL.println(" uS");
}

// REFERENCES:
// 1. Original code and sensor description: https://wiki.seeedstudio.com/Grove-TDS-Sensor/
// 2. Temperature compensation for EC: https://www.aqion.de/site/112Source: https://www.aqion.de/site/112

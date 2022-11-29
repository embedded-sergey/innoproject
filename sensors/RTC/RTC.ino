#include <SPI.h>
#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */

// The setup function runs once when you press reset (CONTROLLINO RST button) or connect power supply 
// (USB or external 12V/24V) to the CONTROLLINO.
void setup() {
 // initialize serial communication at 9600 bits per second
 Serial.begin(9600);
 Controllino_RTC_init(0);
 //Controllino_SetTimeDate(29,2,11,22,18,47,40); // set initial values to the RTC chip
}

// the loop function runs over and over again forever
void loop() {
 int n; 
 Serial.print("Day: ");n = Controllino_GetDay(); Serial.println(n);
 
 Serial.print("WeekDay: ");n = Controllino_GetWeekDay(); Serial.println(n);
 
 Serial.print("Month: ");n = Controllino_GetMonth(); Serial.println(n);

 Serial.print("Year: ");n = Controllino_GetYear(); Serial.println(n);

 Serial.print("Hour: ");n = Controllino_GetHour(); Serial.println(n);

 Serial.print("Minute: "); n = Controllino_GetMinute(); Serial.println(n);

 Serial.print("Second: ");n = Controllino_GetSecond(); Serial.println(n);
 
 delay(5000); 
}

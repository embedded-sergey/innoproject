#include <FastLED.h>
#include <Controllino.h>

#define LED_PIN     9
#define NUM_LEDS    30

CRGB leds[NUM_LEDS];
void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}
void loop() {
  for (int i=0; i<6; i++){
    for (int j=0; j<5; j++){
      if (j==2){
        leds[i*5+2] = CRGB (255, 0, 0); //red
      }
      else if(j==4){
        leds[i*5+4] = CRGB (255, 0, 0); //red
        }
      else{
        leds[i*5+j] = CRGB (35, 0, 255); //blue
      }
    }
  }
FastLED.show();
}

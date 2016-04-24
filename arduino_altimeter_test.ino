#include <Adafruit_BMP085.h>

// LED Skydive Altimeter Sketch by Bodey Marcoccia
// Altitude settings can be modified by changing the four altitude variables near the top of the code.

#include <SFE_BMP180.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

SFE_BMP180 pressure;

#define PIN 2
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

uint32_t blue      = strip.Color(0, 0, 255);
uint32_t cyan      = strip.Color(36, 182, 255);
uint32_t cyan_dim  = strip.Color(0, 20, 20);
uint32_t green     = strip.Color(0, 255, 0);
uint32_t red       = strip.Color(255, 0, 0);
uint32_t violet    = strip.Color(109, 36, 255);
uint32_t white_dim = strip.Color(20, 20, 20);
uint32_t yellow    = strip.Color(255, 255, 0);
uint32_t off       = strip.Color(0, 0, 0);

int exitalt  = 4000; // Set exit altitude.
int breakalt = 1500; // Set breakoff altitude.
int pullalt  = 1000; // Set pull altitude.
int harddeck = 700;  // Set hard deck.
int num_leds = 4;    // Set number of LEDs

double baseline;

int startup = 0;
int descending = 0;

int setLEDColors(int nr_leds, uint32_t color) {
  for(uint16_t i=0; i<nr_leds; i++) {
      strip.setPixelColor(i, color);
  }
  strip.show();
}

int blinkLEDColors(int nr_leds, uint32_t color, int on_time, int off_time) {
  for(uint16_t i=0; i<nr_leds; i++) {
      strip.setPixelColor(i, color);
  }
  strip.show();
  delay(on_time);
  for(uint16_t i=0; i<nr_leds; i++) {
      strip.setPixelColor(i, off);
  }
  strip.show();
  delay(off_time);
}

void setup() {
  Serial.begin(9600);

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while (1);
  }

  baseline = getPressure();
  strip.begin();
  strip.show();
}

void loop() {

  double P = getPressure();
  int agl = pressure.altitude(P, baseline);
  int lastaltitude = agl;

  if (startup == 0) { // Violet through all LEDs on startup. Sets startup variable to 1.
    setLEDColors(num_leds, violet);
    delay(1000);
    setLEDColors(num_leds, off);
    delay(1000);
    startup = 1;
  }

  if (descending == 0 && agl < 300) { // Blinks green every five seconds before 300 AGL.
    blinkLEDColors(1,red,100,5000);
  }

  // Blink green five times when descent starts and proceed to setting altitude lights.
  while (descending == 0) {
    if (lastaltitude - 50 > (agl)) {
      for(uint16_t i=0; i<5; i++) {
        setLEDColors(num_leds, green);
        delay(100);
        setLEDColors(num_leds, off);
      }
      descending = 1;
    } else
    {
      for(uint16_t i=0; i<5; i++) {
        setLEDColors(1, green);
        delay(100);
        setLEDColors(num_leds, off);
      }
      delay(10000);
      lastaltitude = agl;
    }
  }
  
  if (descending == 1 ) {
    if (agl > 3500) {
      setLEDColors(num_leds,blue);
    }
    else if (agl < 3500 && agl > 3000) {
      blinkLEDColors(num_leds,blue,1000,1000);
    }
    else if (agl < 3000 && agl > 2500) {
      setLEDColors(num_leds,green);
    }
    else if (agl < 2500 && agl > 2000) {
      blinkLEDColors(num_leds,green,1000,1000);
    }
    else if (agl < 2000 && agl > 1500) {
      setLEDColors(num_leds,yellow);
    }
    else if (agl < 1500 && agl > 1000) {
      blinkLEDColors(num_leds,violet,1000,1000);
    }
    else if (agl < 1000 && agl > 600) {
      blinkLEDColors(num_leds,violet,300,300);
    }
    else if (agl < 600 && agl > 300) {
      setLEDColors(num_leds,off);
    }
    else if (agl < 300 && agl > 200) {
      setLEDColors(3,green);
    }
    else if (agl < 200 && agl > 100) {
      setLEDColors(2,blue);
    }
    else if (agl < 100) {
      setLEDColors(1,violet);
    }
  }
}

double getPressure() {
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0)
  {

    delay(status);

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

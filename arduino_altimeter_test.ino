// LED Skydive Altimeter Sketch by Martin Hovmöller
// Fork of Bodey Marcoccias sketch http://www.thingiverse.com/thing:631637.
// Altitude settings can be modified by changing the four altitude variables near the top of the code.
//#include <Adafruit_BMP085.h>
#include <EEPROM.h>
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

int exitalt      = 4000; // Set exit altitude.
int breakalt     = 1500; // Set breakoff altitude.
int pullalt      = 1000; // Set pull altitude.
int harddeck     = 700;  // Set hard deck.
int num_leds     = 4;    // Set number of LEDs
int baseline_pin = 9; // Set pin for baseline button.

int eeprom_address = 0;

double baseline;

int startup = 0;
int altreached = 0;

int setLEDColors(int nr_leds, uint32_t color) {
  for(uint16_t i=0; i<nr_leds; i++) {
      strip.setPixelColor(i, color);
  }
  strip.show();
}

int cycleLEDColors(int nr_leds, uint32_t color, int cycle_time) {
  setLEDColors(num_leds,off);
  for(uint16_t i=0; i<nr_leds; i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(cycle_time);
    strip.setPixelColor(i, off);
    strip.show();
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

  // Connect the baseline button to pin 8.
  pinMode(baseline_pin, INPUT_PULLUP);

  strip.begin();
  strip.show();
}

void loop() {
  double a, P;
  P = getPressure();
  int agl = pressure.altitude(P, baseline);
  int calibrateButtonPressed = digitalRead(baseline_pin);

  /* Reversed logic due to build in pullup resistor.
  The reading is low when the button is pressed. */
  if (calibrateButtonPressed == LOW) {
    Serial.println("Sensor value low");
    baseline = getPressure(); 
    Serial.println("baseline = ");
    Serial.println(baseline / 10);
    EEPROM.write(eeprom_address, baseline / 10);
 //   setLEDColors(num_leds,green);
    startup = 0;
    cycleLEDColors(num_leds,green,200);
    delay(1000);
  } else {
    setLEDColors(num_leds,off);
    baseline = EEPROM.read(eeprom_address);
    Serial.println("Sensor value high");
    Serial.println("baseline = ");
    Serial.println(baseline);

    if (startup == 0) { // Violet through all LEDs on startup. Sets startup variable to 1.
      setLEDColors(num_leds, violet);
      delay(1000);
      setLEDColors(num_leds, off);
      startup = 1;
    }
    
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
      setLEDColors(num_leds,red);
    }
    else if (agl < 1000 && agl > 500) {
      blinkLEDColors(num_leds,red,300,300);
    }
  }
}
  
  double getPressure() {
    char status;
    double T, P, p0, a;
  
    pressure.startTemperature();
    pressure.getTemperature(T);
    status = pressure.startPressure(3);
    if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        return (P);
      }
    else Serial.println("error starting temperature measurement\n");
  }

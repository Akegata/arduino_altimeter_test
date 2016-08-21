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

// Define different colors for easier use.
uint32_t blue      = strip.Color(0, 0, 255);
uint32_t cyan      = strip.Color(36, 182, 255);
uint32_t cyan_dim  = strip.Color(0, 20, 20);
uint32_t green     = strip.Color(0, 255, 0);
uint32_t red       = strip.Color(255, 0, 0);
uint32_t violet    = strip.Color(109, 36, 255);
uint32_t white_dim = strip.Color(20, 20, 20);
uint32_t yellow    = strip.Color(255, 255, 0);
uint32_t off       = strip.Color(0, 0, 0);

int num_leds            = 4;    // Set number of LEDs
int powercycles         = 0;
int powercycles_updated = 0;
int startup             = 0;
int blink               = 0;

// Address in the EEPROM where the baseline reading should be stored.
int baseline_address = 1;
double baseline;

struct MyObject {
  double field1;
  byte field2;
  char name[10];
};

// Sets the specified numbers of LEDs to the specifiedcolor.
int setLEDColors(int nr_leds, uint32_t color) {
  for (uint16_t i = 0; i < nr_leds; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Cycles through the LED's, only lighting up one LED at a time.
int cycleLEDColors(int nr_leds, uint32_t color, int cycle_time) {
  setLEDColors(num_leds, off);
  for (uint16_t i = 0; i < nr_leds; i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(cycle_time);
    strip.setPixelColor(i, off);
    strip.show();
  }
  strip.show();
}

// Blinks the specified number of LED's at the specified interval, with the specified color.
int blinkLEDColors(int nr_leds, uint32_t color, int on_time, int off_time) {
  for (uint16_t i = 0; i < nr_leds; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  delay(on_time);
  for (uint16_t i = 0; i < nr_leds; i++) {
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
  
  strip.begin();
  strip.show();
}

void loop() {
  MyObject read_baseline;
  double agl, P;
  P = getPressure();
  EEPROM.get(baseline_address, read_baseline);
  int calibrate = (EEPROM.read(0));
  agl = pressure.altitude(P, read_baseline.field1);

  // On the third power cycle, reset the calibration
  if (calibrate == 2) {
    baseline = getPressure();
    blinkLEDColors(num_leds, red, 500, 500);
    EEPROM.put(baseline_address, baseline);
    Serial.print("Baseline set");
    powercycles_updated = 1;
  }

  // Update the powercycle count.
  else if (powercycles_updated == 0) {
    powercycles = (calibrate + 1);
    EEPROM.write(0, powercycles);
    powercycles_updated = 1;

    cycleLEDColors(num_leds, blue, 200);
    delay(2000);
  }

  // Reset the power cycle count and read the baseline from EEPROM.
  powercycles = (0);
  EEPROM.write(0, powercycles);
  setLEDColors(num_leds, off);
  EEPROM.get(baseline_address, read_baseline);

  Serial.print("Baseline pressure = ");
  Serial.print(read_baseline.field1);
  Serial.print(". Current pressure = ");
  Serial.print(P);
  Serial.print(". agl = ");
  Serial.println(agl);

  // Blink LED's green tbree times to indicate that the altimeter is running.
  if (startup == 0) { // Violet through all LEDs on startup. Sets startup variable to 1.
    while(blink < 3){
      blinkLEDColors(num_leds, green, 100, 100);
      blink++;
    }
    blink = 0;
    startup = 1;
  }

  // Light up or blink the LEDs in different patterns depending on altitude.
  if (agl > 3500) {
    setLEDColors(num_leds, blue);
  }
  else if (agl < 3500 && agl > 3000) {
    blinkLEDColors(num_leds, blue, 1000, 1000);
  }
  else if (agl < 3000 && agl > 2500) {
    setLEDColors(num_leds, green);
  }
  else if (agl < 2500 && agl > 2000) {
    blinkLEDColors(num_leds, green, 1000, 1000);
  }
  else if (agl < 2000 && agl > 1500) {
    setLEDColors(num_leds, yellow);
  }
  else if (agl < 1500 && agl > 1000) {
    setLEDColors(num_leds, red);
  }
  else if (agl < 1000 && agl > 500) {
    blinkLEDColors(num_leds, red, 300, 300);
  }
}

double getPressure()
{
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

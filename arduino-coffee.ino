#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h" //Die MAX6675 Bibliothek
#include <HX711_ADC.h>
#include <EEPROM.h>

// GENERAL
#define SERIALDEBUG false
String coffeebar = "1337 Coffee Bar";
unsigned long printinterval = 200;
boolean brewingInitialized = false;

// PUMP SWITCH
#define PUMPSWITCH_PIN 2
boolean pumprunning = false;

// TIMER
unsigned long starttime = 0;
unsigned long timer = 0;
unsigned long looptime = millis();

// DISPLAY
const int screen_width = 128;  // OLED display width, in pixels
const int screen_height = 32;  // OLED display height, in pixels
int8_t oled_reset = 4;         // Reset pin # (or -1 if sharing Arduino reset pin)
uint8_t screen_address = 0x3C; ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

// TEMPERATURE
const int max6675SO = 8;   // Serial Output am PIN 8
const int max6675CS = 9;   // Chip Select am PIN 9
const int max6675CLK = 10; // Serial Clock am PIN 10
double temp = 0.0;
int currentIndex = 0;
double lastvals[5] = {-1, -1, -1, -1, -1};
MAX6675 ktc(max6675CLK, max6675CS, max6675SO);

// WEIGHT SCALE
float weight = 0;
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5;  //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
long t;

// reset and tare scale
void tare()
{
#if SERIALDEBUG
  Serial.println("start tare");
#endif

  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  //calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom
  long stabilizingtime = 2000;                       // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                              //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
#if SERIALDEBUG
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
#endif
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)

#if SERIALDEBUG
    Serial.println("Startup is complete");
#endif
  }
}

// arduino setup
void setup()
{
#if SERIALDEBUG
  Serial.begin(9600);
#endif

  // signal drops from 5V to 0V while pump is running
  pinMode(PUMPSWITCH_PIN, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, screen_address);

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner
  display.println(coffeebar);
  display.display();

  starttime = millis();

  tare();
}

void displayData(double temp, unsigned long timer, float weight)
{
  display.clearDisplay();
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner

  display.println("1337 Coffee Bar");
  display.println("Temperatur: " + String(temp) + " C");
  display.println("Timer: " + String(timer) + " s");
  display.println("Volumen: " + String(weight) + " ml");
}

void initBrewing()
{
  timer = 0;
  starttime = millis();

  tare();

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner

  display.println("...pump it up!");
  display.display();

  brewingInitialized = true;

  delay(1000);
}

double getTemp()
{
  // Lesen des Temperaturwertes in Grad Celsius
  double readtemp = ktc.readCelsius();

  double sum = readtemp;
  int count = 1;
  for (int i = 0; i < 5; i = i + 1)
  {
    if (lastvals[i] > 0 && !(i == currentIndex))
    {
      sum = lastvals[i];
      count = count + 1;
    }
  }

  lastvals[currentIndex] = readtemp;

  currentIndex = currentIndex + 1;
  if (currentIndex > 4)
  {
    currentIndex = 0;
  }

  return sum / count; //average
}

void loop()
{

  if (millis() - looptime >= printinterval)
  {
    looptime = millis();

    double temp = getTemp();

    if (digitalRead(PUMPSWITCH_PIN) == LOW)
    {
      if (brewingInitialized == false)
      {
        initBrewing();
      }
#if SERIALDEBUG
      Serial.println("TIMER");
      Serial.println(millis());
      Serial.println(starttime);
#endif

      timer = (millis() - starttime) / 1000;
    }
    else
    {
      brewingInitialized = false;
    }

    displayData(temp, timer, weight);

#if SERIALDEBUG
    Serial.print(temp);
    Serial.println("C");
#endif

    display.display();
  }

  //waage
  static boolean newDataReady = 0;
  if (LoadCell.update())
    newDataReady = true;
  // get smoothed value from the dataset:
  if (newDataReady)
  {
    weight = LoadCell.getData();
    //Serial.print("Load_cell output val: ");
    //Serial.println(weight);
    newDataReady = 0;
  }

#if SERIALDEBUG
  Serial.println(weight);
#endif

  delay(20);
}

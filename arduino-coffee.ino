#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h" //Die MAX6675 Bibliothek
#include <HX711_ADC.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int max6675SO = 8;   // Serial Output am PIN 8
int max6675CS = 9;   // Chip Select am PIN 9
int max6675CLK = 10; // Serial Clock am PIN 10

unsigned long starttime = 0;

int schalterpin = 2;

unsigned long timer = 0;

unsigned long looptime = millis();

int printinterval = 200;

boolean pumprunning = false;

boolean brewingInitialized = false;

// Initialisierung der MAX6675 Bibliothek mit
// den Werten der PINs
MAX6675 ktc(max6675CLK, max6675CS, max6675SO);

//init waage
float weight = 0;
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5;  //mcu > HX711 sck pin
//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
long t;

void tara()
{
  Serial.println("start tara");
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  //calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom
  long stabilizingtime = 2000;                       // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                              //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

void setup()
{
  Serial.begin(9600);

  pinMode(schalterpin, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner
  display.println("1337 Coffee Bar");
  display.display();

  starttime = millis();

  tara();
}

void printparam(double value, String paraname)
{
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.println(paraname + ": " + String(value));
}

void initBrewing()
{
  timer = 0;
  starttime = millis();

  tara();

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner

  display.println("...pump it up!");
  display.display();

  brewingInitialized = true;

  delay(1000);
}

void loop()
{

  if (millis() - looptime >= printinterval)
  {
    looptime = millis();

    // Lesen des Temperaturwertes in Grad Celsius
    double temp = ktc.readCelsius();

    if (digitalRead(schalterpin) == LOW)
    {
      if (brewingInitialized == false)
      {
        initBrewing();
      }
      Serial.println("TIMER");
      Serial.println(millis());
      Serial.println(starttime);

      timer = (millis() - starttime) / 1000;
    }
    else
    {
      brewingInitialized = false;
    }

    display.clearDisplay();
    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner

    display.println("1337 Coffee Bar");
    display.println("Temperatur: " + String(temp) + " C");
    display.println("Timer: " + String(timer) + " s");
    display.println("Volumen: " + String(weight) + " ml");

    Serial.print(temp);
    Serial.println("C");

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
Serial.println(weight);
  delay(20);
}

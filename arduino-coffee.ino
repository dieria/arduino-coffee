// GENERAL
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <SPI.h>
#define SERIALOUT false
#define TITLE " 1337 Coffee Bar"

// EEPROM
#include <EEPROM.h>
const int calVal_eepromAdress = 150;
const int aggKp_addr = 0;
const int aggKi_addr = 10;
const int aggKd_addr = 20;
const int setPoint_addr = 30;

// PINS
// Display SDA: D21
// Display SCL: D22
const int HX711_dout = 19; //D19
const int HX711_sck = 18;  //D18
#define pinRelayHeater 25  //D25 Output pin for heater
const int tsic1pin = 26;   //D26
const int tsic2pin = 27;   //D27
const int dallas1pin = 32; //D32
#define PUMPSWITCH_PIN 33  // D33
#define WIFILED 2

// DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
unsigned long msgStartTime = 0;
unsigned long msgDuration = 1000;
unsigned long lastDisplayRefresh = 0;
unsigned long refreshInterval = 500;
const int screen_width = 128;  // OLED display width, in pixels
const int screen_height = 32;  // OLED display height, in pixels
int8_t oled_reset = -1;        // Reset pin # (or -1 if sharing Arduino reset pin)
uint8_t screen_address = 0x3C; ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

// TIMER
unsigned long starttime = 0;
unsigned long brewtimer = 0;
unsigned long looptime = 0;

// WEIGHT SCALE
#include <HX711_ADC.h>
unsigned long lastWeightRefresh = 0;
unsigned long weightRefreshInterval = 20;
boolean scaleError = false;
float weight = 0;
float calibrationValue = 696;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
long t;

// HEATER + PID
#include <DallasTemperature.h>
#include "TSIC.h"
#include <PID_v2.h>
hw_timer_t *heatTimer = NULL;
TSIC Sensor1(tsic1pin, NO_VCC_PIN, TSIC_30x);
TSIC Sensor2(tsic2pin, NO_VCC_PIN, TSIC_30x);

volatile float temperature2 = 0.0; // to V4
unsigned long previousMillistemp;  // initialisation at the end of init()
const unsigned long intervaltempmesds18b20 = 3000;
OneWire oneWire(dallas1pin);         //D32  Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress sensorDeviceAddress;   // arrays to hold device address

uint16_t temperatureRaw = 0;
volatile float temperature = 0.0; // to V2
uint16_t bTemperatureRaw = 0;
volatile float bTemperature = 0.0; // to V3
const unsigned int windowSize = 1000;
volatile unsigned int isrCounter = 0; // counter for ISR
unsigned long windowStartTime;
volatile double pwmOutput;
double setPoint = 102;
double aggKp = 50;
double aggKi = 1;
double aggKd = 1000;
PID_v2 heaterPID(aggKp, aggKi, aggKd, PID::Direct); //, PID::P_On::Measurement);

void IRAM_ATTR onTimer()
{
  // TODO portENTER_CRITICAL / mux variable security

  if (pwmOutput <= isrCounter)
  {
    digitalWrite(pinRelayHeater, LOW);
  }
  else
  {
    digitalWrite(pinRelayHeater, HIGH);
  }

  isrCounter += 10; // += 10 because one tick = 10ms
  if (isrCounter > windowSize)
  {
    isrCounter = 0;
  }
}

// PUMP SWITCH
unsigned long lastPumpRefresh = 0;
unsigned long pumpRefreshInterval = 800;
boolean pumprunning = false;
boolean brewingInitialized = false;

// OTA
#include <ArduinoOTA.h>
#define OTA false         // true = OTA activated, false = OTA deactivated
#define OTAHOST "ESP8266" // Name to be shown in ARUDINO IDE Port
#define OTAPASS "otapass" // Password for OTA updtates
const boolean ota = OTA;
const char *OTAhost = OTAHOST;
const char *OTApass = OTAPASS;

// WIFI
#define WIFI_SSID "BLACK-MESA-NETWORK"   //Enter Wifi Name
#define WIFI_PASS "68266239544296176958" //Enter wifi Password
int wifiFlag = 0;

// BLYNK
#define AUTH "vI8-86-5S3CoN7CNTGwJ9iPu-kk8mmfS" // You should get Auth Token in the Blynk App.
BlynkTimer timer;

BLYNK_CONNECTED()
{
  // Request the latest state from the server
  Blynk.syncVirtual(V5);

  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);
  Blynk.syncVirtual(V12);

  EEPROM.begin(1024);
  EEPROM.put(aggKd_addr, aggKp);
  EEPROM.put(aggKi_addr, aggKi);
  EEPROM.put(aggKd_addr, aggKd);
  EEPROM.put(setPoint_addr, setPoint);
  EEPROM.commit();

  Serial.println("BLYNK CONNECTED");
}

BLYNK_WRITE(V5)
{
  setPoint = param.asDouble();
  heaterPID.Setpoint(setPoint);
}

BLYNK_WRITE(V10)
{
  aggKp = param.asDouble();
  heaterPID.SetTunings(aggKp, aggKi, aggKd);
}

BLYNK_WRITE(V11)
{
  aggKi = param.asDouble();
  heaterPID.SetTunings(aggKp, aggKi, aggKd);
}

BLYNK_WRITE(V12)
{
  aggKd = param.asDouble();
  heaterPID.SetTunings(aggKp, aggKi, aggKd);
}

// start up the display
void initDisplay()
{
  display.begin(SSD1306_SWITCHCAPVCC, screen_address);

  msgStartTime = millis();

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner
  display.println(TITLE);
  display.display();
}

// update the display with all values etc
void displayData(double temp, unsigned long timer, float weight)
{
  if (millis() - lastDisplayRefresh > refreshInterval && millis() - msgStartTime > msgDuration)
  {
    display.clearDisplay();
    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner

    display.println(TITLE);
    display.println(" " + String(temp, 1) + " / " + String(setPoint, 1) + " C");
    display.println(" Timer:   " + String(timer) + " s");
    if (!scaleError)
    {
      display.println(" Vol.:   " + String(weight, 0) + " ml");
    }
    else
    {
      display.println(" SCALE ERROR");
    }
    display.display();

    lastDisplayRefresh = millis();
  }
}

// initialize the scale
void initScale()
{
  EEPROM.get(calVal_eepromAdress, calibrationValue); //set by calibration ino script
  if (isnan(calibrationValue))
  {
    calibrationValue = 1816.15;
    Serial.println("Scale not calibrated! Fallback...");
  }
  else
  {
    Serial.println("Scale calibration value read from EEPROM");
  }

  LoadCell.begin();
  long stabilizingtime = 1000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;        //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    scaleError = true;
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
  }
}

// set the scale to zero
void tareScale()
{
  Serial.println("start tare");
  LoadCell.tare(); //blocking tare
}

// reset scale and timer and display a pump start message
void initBrewing()
{
  brewtimer = 0;
  starttime = millis();

  brewingInitialized = true;

  msgStartTime = millis();

  display.clearDisplay();
  display.setTextSize(1.7);            // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Start at top-left corner

  display.println("...pump it up!");
  display.display();

  tareScale();
}

// update the weight and save it to weight variable when ready
void updateWeight()
{
  if (millis() - lastWeightRefresh > weightRefreshInterval)
  {
    //waage
    static boolean newDataReady = 0;
    if (LoadCell.update())
      newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady)
    {
      weight = LoadCell.getData();
      newDataReady = false;
      //Serial.println("get weight complete "+String(weight));
    }
    lastWeightRefresh = millis();
  }
}

void checkBlynkStatus()
{ // called every 3 seconds by SimpleTimer

  //Serial.println("BLYNK CHECK STATUS");

  bool isconnected = Blynk.connected();
  if (isconnected == false)
  {
    wifiFlag = 1;
    digitalWrite(WIFILED, LOW); //Turn off WiFi LED
  }
  if (isconnected == true)
  {
    wifiFlag = 0;
    digitalWrite(WIFILED, HIGH); //Turn on WiFi LED
  }

  //updateTemperature();

  delay(50);
  Serial.print("Boiler Temp: ");
  Serial.println(temperature);
  delay(50);
  Serial.print("Brew Temp: ");
  Serial.println(bTemperature);
  delay(50);
  Serial.print("Boiler Bottom Temp: ");
  Serial.println(temperature2);
  delay(50);
  Serial.print("reg: ");
  Serial.println(pwmOutput);

  Blynk.virtualWrite(V2, temperature);
  Blynk.virtualWrite(V3, bTemperature);
  Blynk.virtualWrite(V4, temperature2);
}

boolean updateTemperature()
{
  boolean tempUpdated = false;
  if (Sensor1.getTemperature(&temperatureRaw))
  {
    //Serial.print("uint_16: ");
    //Serial.println(temperatureRaw);
    const float oldTemp1 = temperature;
    temperature = Sensor1.calc_Celsius(&temperatureRaw);
    if (temperature > 160 || temperature < 5)
    {
      temperature = oldTemp1;
    }
    else
    {
      tempUpdated = true;
    }
    //Serial.print("Temperature: ");
    //Serial.println(temperature);
    //Serial.println(" C");
  }

  // ! CURRENTLY NOT INSTALLED
  // if (Sensor2.getTemperature(&bTemperatureRaw))
  // {
  //   //Serial.print("uint_16: ");
  //   //Serial.println(bTemperatureRaw);
  //   const float oldTemp2 = bTemperature;
  //   bTemperature = Sensor2.calc_Celsius(&bTemperatureRaw);
  //   if (bTemperature > 160 || bTemperature < 5)
  //   {
  //     bTemperature = oldTemp2;
  //   }
  //   //Serial.print("Temperature: ");
  //   //Serial.println(bTemperature);
  //   //Serial.println(" C");
  // }

  unsigned long currentMillistemp = millis();
  if (currentMillistemp - previousMillistemp >= intervaltempmesds18b20)
  {
    const float oldTemp3 = temperature2;
    previousMillistemp = currentMillistemp;
    sensors.requestTemperatures();
    temperature2 = sensors.getTempCByIndex(0);
    if (temperature2 > 160 || temperature2 < 5)
    {
      temperature2 = oldTemp3;
    }
  }

  return tempUpdated;
}

void checkPumpSwitch()
{
  if (millis() - lastPumpRefresh >= pumpRefreshInterval)
  {
    if (digitalRead(PUMPSWITCH_PIN) == LOW)
    {
      if (brewingInitialized == false)
      {
        initBrewing();
      }
      pumprunning = true;
    }
    else
    {
      brewingInitialized = false;
      pumprunning = false;
    }
    lastPumpRefresh = millis();
  }
}

void updateBrewTimer()
{
  brewtimer = (millis() - starttime) / 1000;
}

void initDefaults()
{
  EEPROM.begin(1024); // open eeprom
  double dummy;       // check if eeprom values are numeric (only check first value in eeprom)
  EEPROM.get(0, dummy);
  Serial.println("check eeprom 0x00 in dummy: ");
  Serial.println(dummy);
  if (!isnan(dummy))
  {
    EEPROM.get(aggKp_addr, aggKp);
    EEPROM.get(aggKi_addr, aggKi);
    EEPROM.get(aggKd_addr, aggKd);
    EEPROM.get(setPoint_addr, setPoint);
  }
  else
  {
    Serial.println("Nothing in EEPROM");
  }
  // eeeprom schließen
  EEPROM.commit();
}

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(115200);

  starttime = millis();

  pinMode(pinRelayHeater, OUTPUT);

  // signal drops from 5V to 0V while pump is running
  pinMode(PUMPSWITCH_PIN, INPUT_PULLUP);

  initDefaults();
  initDisplay();
  initScale();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setHostname("arduino-coffee");

  timer.setInterval(3000L, checkBlynkStatus); // check if Blynk server is connected every 3 seconds
  Blynk.config(AUTH);

  heatTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(heatTimer, &onTimer, true);
  timerAlarmWrite(heatTimer, 10000, true); // 10 ms
  timerAlarmEnable(heatTimer);

  // tell the PID to range between 0 and the full window size
  heaterPID.SetOutputLimits(0, windowSize);

  // init timers
  unsigned long currentTime = millis();
  previousMillistemp = currentTime;

  // init dallas sensors (twice for better stability)
  sensors.begin();
  sensors.begin();

  sensors.requestTemperatures();
  temperature2 = sensors.getTempCByIndex(0);

  updateTemperature();

  // turn the PID on
  heaterPID.Start(temperature, 0, setPoint);
}

// the loop function runs over and over again forever
void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    //Serial.println("WiFi Not Connected");
  }
  else
  {
    ArduinoOTA.handle(); // For OTA
    // Disable interrupt it OTA is starting, otherwise it will not work

    //Serial.println("WiFi Connected");
    Blynk.run();
  }

  timer.run(); // Initiates SimpleTimer

  boolean tempUpdate = updateTemperature();
  if (tempUpdate == true)
  {
    pwmOutput = heaterPID.Run(temperature);

    if (temperature >= setPoint && pwmOutput > 300)
    {
      pwmOutput = 0;
    }
    else if (temperature < setPoint - 15)
    {
      pwmOutput = 1000.0;
    }

    // ! TEMPERATURE ERROR: Disable Heating
    if (temperature == 0 || temperature > 150)
    {
      pwmOutput = 0;
      Serial.println("TEMPERATURE OUTSIDE RANGE, DISABLING HEATING");
    }
  }

  updateWeight();
  checkPumpSwitch();

  displayData(temperature, brewtimer, weight);

  if (pumprunning == true)
  {
    updateBrewTimer();
  }
}

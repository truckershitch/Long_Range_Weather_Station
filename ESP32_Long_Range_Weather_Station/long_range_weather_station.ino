#include <WiFi.h>
// #include <HTTPClient.h>
#include <TelnetStream.h>
#include <ArduinoJson.h>

#define LED_PIN 22
#define PWR_PIN 2     //will  be used to power all the sensors when HIGH

//servo
// Values for TowerPro SG90 small servos; adjust if needed
#define COUNT_LOW 2200
#define COUNT_HIGH 7100
#define TIMER_WIDTH 16
#include "esp32-hal-ledc.h"
#define SERVO_PIN 33

//sensors selection
#define HAS_HX711       //uncomment for weighing rain gauge
#define HAS_ANEMOMETER  //uncomment for anemometer
#define HAS_AS5600      //uncomment for Wind Direction sensor
#define HAS_BME280      //uncomment for temperature + pressure + humidity BME280 sensor
//#define HAS_DS18B20     //uncomment for DS18B20 temperature sensor
//#define HAS_DHT22       //uncomment for DTH22 temperature + humidity sensor
#define ENABLE_WIFI     //uncomment to enable wifi access (and WiFiManager)
// #define CONNECT_WIFI   //uncomment to enable wifi debugging (avoid this when ESPNow in use)
//#define HAS_MQTT        //uncomment to enable MQTT (need to uncomment ENABLE_WIFI also)
#define HAS_LORA        //uncomment to enable LORAWAN radio
#define HAS_LDR         //uncomment to enable light dependent resistor

#ifdef HAS_MQTT
#include <WiFiClient.h>
#include <PubSubClient.h>
#endif

#ifdef HAS_LORA
#include <SPI.h>
#include <LoRa.h>
//define the pins used by the LoRa transceiver module
#define SCK 26
#define MISO 35
#define MOSI 25
#define CS 27
#define RST 14
#define DIO0 12
//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 915E6
int timeCmd[6];
bool timeSetInLoop = false;
byte localAddr = 0xBB;
byte destAddr = 0xAA;
#endif

float humidity = 0;
float temperature = 0;
float outsideTemperature = 0;
float pressure = 0 ;
float windAngle = 0;
float windSpeed = 0;
float windSpeedMax = 0;
long windTimeOut = 0;     //used in loop to sample winsSpeedMax every 2s
int smooth = 1000;        //acquire smooth*values for each ADC
float Vin = 0.;           //input Voltage (solar panel voltage)
int sensorsGetTime = 15000; //ms to acquire sensors in worst case

//DHT sensor (temp + humidity)
#define DHTPIN 5             // Digital pin connected to the DHT sensor
#ifdef HAS_DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTTYPE    DHT22     // DHT 22 (AM2302) --> https://learn.adafruit.com/dht/overview
DHT dht(DHTPIN, DHTTYPE);
#endif

//temperature sensor
#define ONE_WIRE_BUS 13       // Data wire is plugged into pin 13 on the ESP32
#define TEMP_OFFSET -0.3
#ifdef HAS_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempScale1(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif

//scale
#define PIN_CLOCK  32        //output to generate clock on Hx711
#define PIN_DOUT   34        //input Dout from Hx711

long calibZero = 0;  //No load sensor Output
long calib = 130968;          //sensor output - calibZero for Weight calibration --> will be auto calibrated later
int calibWeight = 300;         //weight at which calinration is done --> expressed in gramsx10. eg 335 means 33.5g -- 6 nickels @ 5g/nickels
float AverageWeight = 0;
float CurrentRawWeight = 0;
float RealTimeWeight = 0;
int iWeight;
float rainWeight;
float rain;
boolean hasEmptiedBucket = false;



#define FILTER_SAMPLES   50              // filterSamples should  be an odd number, no smaller than 3
#define REJECT_RATIO     25               //points to reject % left and right before averaging
float weightSmoothArray [FILTER_SAMPLES];   // array for holding raw sensor values for sensor1

//anemometer
#define HALL_OUT_PIN 4
int tops = 0;       //nb tops when anemometer rotates
long hallTimeout;   //to debounce
int anemometerMeasuringTime;

//WindDirection
#define SDA_PIN 16
#define SCL_PIN 17
#include "Wire.h"
#ifdef HAS_AS5600
#include "AS5600.h"

AS5600 as5600;   //  use default Wire
#endif

float calibAngle = 0; //raw value when pointing to North

//BME280 (temperature, pressure, humidity)
//uses the wire library already included
#ifdef HAS_BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
#endif

#define VIN_PIN 36    //ADC pin for solar panel voltage measurement


// extern DataReading myCtrlData[256];
// extern uint8_t myCtrl;
//end FDRS

long timeOut = 0;
long telnetTimeOut;
int sendStateTimeOut = 80000;
int counter;
// int forceSleep = 0;
boolean hasReceivedCmd = false;
boolean hasReceivedTime = false;
boolean hasSentData = false;

enum {idle, waitingTime, sendingSensor, sleeping};        // 0: idle, 1: sendingTime, 2: waitingSensor, 3: sleeping
int GtwStatus = idle;

//float M1;                           // Moisture sensors 1 (note that M2 on the PCB is used to calibrate sensors)

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int timeToSleep;                    /* Time ESP32 will go to sleep (in seconds) */



//RTC DS1302
#include <Ds1302.h>
#define ENA_PIN 18
#define CLK_PIN 19
#define DAT_PIN 23
// DS1302 RTC instance
Ds1302 rtc(ENA_PIN, CLK_PIN, DAT_PIN);


const static char* WeekDays[] =
{
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday",
  "Sunday"
};


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;
boolean configWiFi = false;
boolean hasNtpTime = false;                 //UTC time not acquired from NTP
int timeZone = -5;   //set to UTC
const int dst = 3600;

//these variable remain in RTC memory even in deep sleep or after software reset (https://github.com/espressif/esp-idf/issues/7718)(https://www.esp32.com/viewtopic.php?t=4931)
RTC_NOINIT_ATTR boolean hasRtcTime = false;   //UTC time not acquired from smartphone

RTC_NOINIT_ATTR int hours;
RTC_NOINIT_ATTR int seconds;
RTC_NOINIT_ATTR int tvsec;
RTC_NOINIT_ATTR int minutes;
RTC_NOINIT_ATTR int days;
RTC_NOINIT_ATTR int months;
RTC_NOINIT_ATTR int years;

//reseted after software reset
//RTC_DATA_ATTR boolean hasRtcTime = false;   //will only survice to deepsleep reset... not software reset
RTC_DATA_ATTR float previousRainWeight = 0;


//time
#include <TimeLib.h>


boolean touchWake = false;
boolean resetWake = false;
touch_pad_t touchPin;
int threshold = 45; //Threshold value for touchpads pins
bool touch9detected = false;  //touch9
bool touch3detected = false;  //touch3 used to calibrate sensors (hold it while reseting)
bool touch0detected = false;  //touch0 used to launch WifiManager (hold it while reseting)

//Preferences
#include <Preferences.h>
Preferences preferences;

#define DEBUG_WIFI   //debug Wifi
#define DEBUG_SLEEP
#define DEBUG_UDP    //broadcast info over UDP
#define DEBUG_PREFS  //debug preferences
#define DEBUG_VIN
#define DEBUG
#define RAW_WEIGHT_DEBUG
#define PREFERENCES_DEBUG


//WifiManager
#ifdef ENABLE_WIFI
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

//define your default values here, if there are different values in config.json, they are overwritten.
char ascMargin[3];  //should contain 2 char "20" margin in minutes
String strAscMargin; //same in String


//flag for saving data
bool shouldSaveConfig = true; // TH Mod - was false

//callback notifying us of the need to save config
void saveConfigCallback ()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
#endif

#ifdef HAS_LDR
#define LDR 39
int ldr = 0;
#endif

#define DBG(a) Serial.println(a);

//********************
//code starts here
//********************

#ifdef HAS_ANEMOMETER
void IRAM_ATTR hall_ISR()    //hall sensor interrupt routine
{
  if ((millis() - hallTimeout) > 10)
  {
    hallTimeout = millis();
    tops++;
    //Serial.println( tops);  //should comment this line to avoid crashes
  }
}
#endif

#include "rom/rtc.h"
void print_reset_reason(int reason) //Print last reset reason of ESP32
{
  switch ( reason)
  {
    case 1 :                                                    //Vbat power on reset
      Serial.println ("POWERON_RESET");
      resetWake = true;
      hasRtcTime = false;                                       //this is the only reset case where RTC memory persistant variables are wiped
      break;
    case 3 : Serial.println ("SW_RESET"); break;                //Software reset digital core
    case 4 : Serial.println ("OWDT_RESET"); break;              //Legacy watch dog reset digital core
    case 5 :                                                    //Deep Sleep reset digital core
      Serial.println ("DEEPSLEEP_RESET");
      print_wakeup_reason();
      break;
    case 6 : Serial.println ("SDIO_RESET"); break;              //Reset by SLC module, reset digital core
    case 7 : Serial.println ("TG0WDT_SYS_RESET"); break;        //Timer Group0 Watch dog reset digital core
    case 8 : Serial.println ("TG1WDT_SYS_RESET"); break;        //Timer Group1 Watch dog reset digital core
    case 9 : Serial.println ("RTCWDT_SYS_RESET"); break;        //RTC Watch dog Reset digital core
    case 10 : Serial.println ("INTRUSION_RESET"); break;        //Instrusion tested to reset CPU
    case 11 : Serial.println ("TGWDT_CPU_RESET"); break;        //Time Group reset CPU
    case 12 : Serial.println ("SW_CPU_RESET"); break;           //Software reset CPU
    case 13 : Serial.println ("RTCWDT_CPU_RESET"); break;       //RTC Watch dog Reset CPU
    case 14 : Serial.println ("EXT_CPU_RESET"); break;          //for APP CPU, reseted by PRO CPU
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET"); break; //Reset when the vdd voltage is not stable
    case 16 : Serial.println ("RTCWDT_RTC_RESET"); break;       //RTC Watch dog reset digital core and rtc module
    default : Serial.println ("NO_MEAN");
  }
}

void print_wakeup_reason()  //deepSleep wake up reason
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      Serial.println("Wakeup caused by touchpad");
      touchWake = true;
      print_wakeup_touchpad();
      break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void print_wakeup_touchpad() {
  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch (touchPin)
  {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 0"); break;
    case 2  : Serial.println("Touch detected on GPIO 2"); break;
    case 3  : Serial.println("Touch detected on GPIO 15"); break;
    case 4  : Serial.println("Touch detected on GPIO 13"); break;
    case 5  : Serial.println("Touch detected on GPIO 12"); break;
    case 6  : Serial.println("Touch detected on GPIO 14"); break;
    case 7  : Serial.println("Touch detected on GPIO 27"); break;
    case 8  : Serial.println("T9 detected "); break;  //GPIO32
    case 9  : Serial.println("T8 detected "); break;  //GPIO33
    default : Serial.println("Wakeup not by touchpad"); break;
  }
}
void display_time(void)
{
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.print(day());
  Serial.print(" at ");
  // Serial.print(hour());
  Serial.printf("%02d:", hour());
  // Serial.print(minute());
  Serial.printf("%02d:", minute());
  // Serial.println(second());
  Serial.printf("%02d\n", second());
}


void setup() {
  pinMode(LED_PIN, OUTPUT);      // initialize digital pin 22 as an output.
  digitalWrite(LED_PIN, HIGH);
  pinMode(PWR_PIN, OUTPUT);      // initialize digital pin 0 as an output
  digitalWrite(PWR_PIN, LOW);   //all sensors are Off
  sensorsGetTime = millis();

  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("*****************************************");
  Serial.print("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("*****************************************");

  if (resetWake)
  {
    Serial.print("touch3 : ");
    Serial.print(touchRead(T3));
    Serial.print("/");
    Serial.println(threshold);
    if (touchRead(T9) < threshold) touch9detected = true; //detect touchpad for T9
    if (touchRead(T3) < threshold) touch3detected = true; //detect touchpad for CONFIG_PIN
    //if (touchRead(T2) < threshold) touch0detected = true; //detect touchpad for wifiManager
    if (touchRead(T0) < threshold) touch0detected = true; //detect touchpad for wifiManager
  }

  //Preferences -- saved to flash memory
  preferences.begin("Rezodo", false);
  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  calibZero = preferences.getLong("calibZero", 0);
  calib = preferences.getLong("calib", 0);
  calibAngle = preferences.getFloat("calibAngle", 0);   //raw value of AS5600 when pointing to North

  timeToSleep = preferences.getInt("timeToSleep", 4);
  ssid = preferences.getString("ssid", "");         // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");
  configWiFi = preferences.getBool("configWiFi", false);

#ifdef PREFERENCES_DEBUG
  Serial.println("_________________");
  Serial.print("calib0 HX711 : ");
  Serial.println(calibZero);
  Serial.print("calib HX711 : ");
  Serial.println(calib);
  Serial.print("calibAngle: ");
  Serial.println(calibAngle);
  Serial.print("configWiFi: ");
  Serial.println(configWiFi);
  Serial.print("timeToSleep : ");
  Serial.println(timeToSleep);
  Serial.println("_________________");
#endif

  //enable deepsleep for ESP32
  //  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH); //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);                 //allow timer deepsleep
  //esp_sleep_enable_touchpad_wakeup();                                           //allow to wake up with touchpads

  //connect to WiFi
#ifdef ENABLE_WIFI
  if (configWiFi) { // parameter sent from LoRa Base Station radio packet (sends to 1 to enable)
    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    //WiFiManagerParameter custom_ascMargin("margin", "margin", ascMargin, 3);

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    //add all your parameters here
    //  const char* z2 = "<p>calib (gx10)</p>";
    //  WiFiManagerParameter custom_text2(z2);
    //  wifiManager.addParameter(&custom_ascMargin);
    //  wifiManager.addParameter(&custom_text2);

    //reset settings - for testing
    //wifiManager.resetSettings();

    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    wifiManager.setTimeout(300);

    if (resetWake && touch0detected || configWiFi) { //then launch WifiManager
      //fetches ssid and pass and tries to connect
      //if it does not connect it starts an access point with the specified name
      //here  "AutoConnectAP"
      //and goes into a blocking loop awaiting configuration

      if (configWiFi) preferences.putBool("configWiFi", false); // avoid infinite loop

      if (!wifiManager.startConfigPortal("TH WeatherStation")) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
      }
    }

    //  //save the custom WifiManager's parameters if needed
    if (shouldSaveConfig) {
      Serial.println("saving Wifi credentials ");
      //read updated parameters
      //    strcpy(ascMargin, custom_ascMargin.getValue());
      //    calibWeight = atoi(ascMargin);
      //    preferences.putInt("calibWeight", calibWeight);
      //Serial.println(ascMargin);
      preferences.putString("password", WiFi.psk());
      preferences.putString("ssid", WiFi.SSID());
      ESP.restart();
      delay(5000);
    }
  }
#endif // ENABLE_WIFI


#ifdef DEBUG
  Serial.println(" == > acquiring sensors");
#endif
  digitalWrite(LED_PIN, HIGH);  //led off to save juice
  digitalWrite(PWR_PIN, HIGH);  //all sensors are On

  //panel voltage
  //ADC
  //analogSetClockDiv(255);
  //analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);        //Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db=0, ADC_2_5db=1, ADC_6db=2, ADC_11db=3

  for (int i = 0; i < smooth; i++) Vin += analogRead(VIN_PIN);
  Vin = Vin / smooth ;
#ifdef DEBUG_VIN
  Serial.print("Vin ");
  Serial.print(Vin);
  Serial.print(" / ");
#endif
  Vin = volts(Vin);
#ifdef DEBUG_VIN
  Serial.println(Vin);
#endif

  //anemometer
#ifdef HAS_ANEMOMETER
#ifdef DEBUG
  Serial.println("anemometer enabled");
#endif
  pinMode(HALL_OUT_PIN, INPUT_PULLUP);
  attachInterrupt(HALL_OUT_PIN, hall_ISR, FALLING);   //will count tops on Anemometer hall Sensor
  hallTimeout = millis();
  tops = 0;
  anemometerMeasuringTime = millis();
#endif

  //WindDirection
  Wire.begin(SDA_PIN, SCL_PIN);
#ifdef HAS_AS5600

  //  as5600.begin(4);  //  set direction pin.
  //  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  if (b)  //if connection OK
  {
    Serial.print("wind Direction: ");
    windAngle = getWindAngle();
    Serial.println(windAngle);
  }

#endif

  //BME280
#ifdef HAS_BME280

  unsigned status;

  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else
  {
    Serial.println("BME280 enabled");
    Serial.print("Temperature = ");
    temperature = bme.readTemperature();
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Pressure = ");
    pressure = bme.readPressure() / 100.0F;
    Serial.print(pressure);
    Serial.println(" hPa");

    //    Serial.print("Approx. Altitude = ");
    //    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //    Serial.println(" m");

    Serial.print("Humidity = ");
    humidity = bme.readHumidity();
    Serial.print(humidity);
    Serial.println(" %");
  }
#endif

#ifdef HAS_LDR
  readLDR();

  Serial.printf("LDR Value: %d\n", ldr);
#endif

  //outside temperature sensor
#ifdef HAS_DS18B20
  tempScale1.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  outsideTemperature = 0;
  int i;
  int kk;
  for (i = 0; i < 2; i++) {            //read twice to warm up
    tempScale1.requestTemperatures();   //needed as first reading may be stuck to 25°
    kk = tempScale1.getTempCByIndex(0);
  }
  for (i = 0; i < 20; i++) {
    tempScale1.requestTemperatures();
    outsideTemperature += tempScale1.getTempCByIndex(0);
  }
  outsideTemperature /= i;
  outsideTemperature += TEMP_OFFSET; //offset measured...
#ifdef DEBUG
  Serial.print("outside temperature = ");
  Serial.print(outsideTemperature);
  Serial.println(" °C");
#endif
#endif

  //scale init
#ifdef HAS_HX711
  pinMode(PIN_CLOCK, OUTPUT); // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);   //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);     //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);  // initialize digital pin 5 as an input.(data Out)

  GetRawWeight();       //HX711 will sleep after weight acquisition
  if (touch3detected)
  {
    Serial.print ("-=[M3 Touched]=- Calibrating HX711... ");
    calib = CurrentRawWeight;
    Serial.println(CurrentRawWeight);
    preferences.putLong("calib", calib);
    emptyBucket();
  }

  AverageWeight = (calibZero - CurrentRawWeight) * calibWeight / (calibZero - calib);
  rainWeight = AverageWeight / 10;
  if ((rainWeight - previousRainWeight)  < -.2 ) //something not normal bucket has lost more than 0.2g !
  {
#ifdef DEBUG
    Serial.print ("bucket has lost more than 0.2g... ");
    Serial.println(rainWeight);
#endif
    emptyBucket();
    rain = 0;
  }
  else if (resetWake)
  {
#ifdef DEBUG
    Serial.println ("manual reset... ");
#endif
    emptyBucket();              //we have lost previousRainWeight... must empty the bucket
    rain = 0;
  }
  else                                  //possibly some rain
  {
    rain = rainWeight - previousRainWeight;
    if (rain < 0) rain = 0;
    if (rainWeight > 50)
    {
#ifdef DEBUG
      Serial.print ("bucket full... ");
      Serial.println(rainWeight);
#endif
      emptyBucket();              //bucket is almost full
    }
    else if ((hours == 12) && (minutes == 0))
    {
#ifdef DEBUG
      Serial.print ("it's noon... ");
      Serial.println(rainWeight);
#endif
      emptyBucket();
    }
  }
#ifdef DEBUG
  Serial.print("rain value : ");
  Serial.println(rain);
#endif
  previousRainWeight = rainWeight;
  digitalWrite(PIN_CLOCK, HIGH);  //go to sleep mode
#endif  //HAS_HX711

  //DHT22
#ifdef HAS_DHT22
  dht.begin();      // Reading temperature or humidity takes about 250 milliseconds!
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) // Check if any reads failed and exit early (to try again).
  {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else
  {
    Serial.print(F("Humidity : "));
    Serial.print(humidity);
    Serial.print(F(" %  Temperature : "));
    Serial.print(temperature);
    Serial.println(F("°C "));
  }
#endif  //HAS_DHT22

  //
  //
  //  //soil moisture probes initialization
  //  Serial.println("measuring moisture");
  //  touch_pad_init();
  //  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  //  touch_pad_config(TOUCH_PAD_NUM1, 0); //T1
  //
  //  //M1 read touch output
  //  uint16_t output;
  //  touch_pad_read(TOUCH_PAD_NUM1, &output);  //T1 or M1
  //  M1 = float(output);

  Serial.println("==> end acquisition sensors");

  // initialize the RTC
  rtc.init();

#if defined (ENABLE_WIFI) && defined (CONNECT_WIFI)
  Serial.println("==> connect to Wifi");
  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;
  hasNtpTime = false;
  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 10000)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);


  if (hasWifiCredentials) {
    TelnetStream.begin(); //used to debug over telnet

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //init and get the time
    Serial.println("trying to get time 1");
    configTime(timeZone * 3600, dst, "pool.ntp.org");
    printLocalTime();

    //init and get the time
    Serial.println("trying to get time 2");   //call it twice to have a well synchronized time on soft reset... Why ? bex=caus eit works...
    delay(2000);
    configTime(timeZone * 3600, dst, "pool.ntp.org");
    printLocalTime();

    //disconnect WiFi as it's no longer needed
    //  WiFi.disconnect(true);
    //  WiFi.mode(WIFI_OFF);


    if (hasNtpTime) {  //set the time with NTP info
      time_t now;
      struct tm * timeinfo;
      time(&now);
      timeinfo = localtime(&now);

      years = timeinfo->tm_year + 1900;   //https://mikaelpatel.github.io/Arduino-RTC/d8/d5a/structtm.html
      months = timeinfo->tm_mon + 1;
      days = timeinfo->tm_mday;
      hours = timeinfo->tm_hour - timeZone;
      minutes = timeinfo->tm_min;
      seconds = timeinfo->tm_sec;

      //set ESP32 time manually (hr, min, sec, day, mo, yr)
      setTime(hours, minutes, seconds, days, months, years);
      Serial.print("time after ntp: ");
      display_time();

      struct timeval current_time;        //get ESP32 RTC time and save it
      gettimeofday(&current_time, NULL);
      tvsec  = current_time.tv_sec ;      //seconds since reboot
      hasRtcTime = true;                  //now ESP32 RTC time is also initialized

      // set DS1302 RTC time
      Ds1302::DateTime dt;
      dt.year = years  % 2000;
      dt.month = months;
      dt.day = days;
      dt.hour = hours;
      dt.minute = minutes;
      dt.second = seconds;
      if (rtc.isHalted()) DBG("RTC is halted...");
      rtc.setDateTime(&dt);
    }
  }
#else
  //disconnect WiFi as it's not needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif //ENABLE_WIFI


  // test if clock is halted and set a date-time (see example 2) to start it
  if (rtc.isHalted()) {
    DBG("RTC is halted...");
    if (hasRtcTime) {
      Serial.print("use time from ESP32 RTC: ");
      struct timeval current_time;
      gettimeofday(&current_time, NULL);
      // Serial.printf("seconds : %ld\nmicro seconds : %ld", current_time.tv_sec, current_time.tv_usec);
      //Serial.printf("seconds stored : %ld\nnow seconds : %ld\n", tvsec, current_time.tv_sec);
      int sec  = seconds - tvsec + current_time.tv_sec ;
      sec = hours * 3600 + minutes * 60 + sec;
      int ss = sec % 60;
      sec = sec / 60;
      int mm = sec % 60;
      sec = sec / 60;
      int hh = sec % 24;
      int dd = days + sec / 24;
      //set time manually (hr, min, sec, day, mo, yr)
      setTime(hh, mm, ss, dd, months, years);
      display_time();
    }
  }
  else {
    // get the current time
    Ds1302::DateTime now;
    rtc.getDateTime(&now);
    years = now.year + 2000;
    months = now.month;
    days = now.day;
    hours = now.hour;
    minutes = now.minute ;
    seconds = now.second;
    setTime(hours, minutes, seconds, days, months, years); //set ESP32 time manually
    DBG("using time from DS1302 RTC: ");
    display_time();
    struct timeval current_time;       //get ESP32 RTC time and save it
    gettimeofday(&current_time, NULL);
    tvsec  = current_time.tv_sec ;      //seconds since reboot (now stored into RTC RAM
    hasRtcTime = true;                  //now ESP32 RTC time is also initialized
  }

  timeOut = 0;
  delay(10);                            //to avoid loosing serialPrint...

  DBG("==> start Weather Station full GTW1");

  if (hasEmptiedBucket == false) delay(14000);  //14s is time to empty bucket
  sensorsGetTime = millis() - sensorsGetTime; //now we know how long it takes to acquire sensors
  sensorsGetTime = min(sensorsGetTime, 30000);
  Serial.print("time spent into setup : sensorsGetTime (ms) ");
  Serial.println(sensorsGetTime);

#ifdef HAS_ANEMOMETER
  windSpeed = getAnemometer();
  windSpeedMax = windSpeed;
  windTimeOut = millis();
#endif

#ifdef HAS_LORA
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, CS);
  //setup LoRa transceiver module
  LoRa.setPins(CS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initialized OK!");
#endif
}

#ifdef HAS_HX711
void emptyBucket(void)
{

  //servo to empty the bucket
  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(SERVO_PIN, 1);   // SERVO_PIN assigned to channel 1
  //move servo
  Serial.println("emptying bucket");
  ledcWrite(1, COUNT_LOW);
  delay(2000);
  ledcWrite(1, COUNT_HIGH);
  delay(1000);
  ledcDetachPin(SERVO_PIN);
  pinMode(SERVO_PIN, INPUT);
  delay(1000);
  GetRawWeight();                 //recalibrate zero value after emptying the bucket
  calibZero = CurrentRawWeight;
  preferences.putLong("calibZero", calibZero);
  Serial.print("calibZero = ");
  Serial.println(calibZero);
  previousRainWeight = 0;
  hasEmptiedBucket = true;
}
#endif

void printLocalTime() //check if ntp time is acquired and print it
{
  struct tm timeinfo;
  hasNtpTime = true;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    hasNtpTime = false;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); //https://www.ibm.com/docs/en/workload-automation/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
}

#ifdef HAS_AS5600
float getWindAngle(void) {
  float angleValue = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
  if (touch3detected) //calibrate sensors
  {
    Serial.println ("-=[M3 Touched]=- Calibrating Wind Direction");
    calibAngle = angleValue;
    preferences.putFloat("calibAngle", calibAngle);
  }
  else
  {
    angleValue = as5600.rawAngle() * AS5600_RAW_TO_DEGREES - calibAngle;
    angleValue += 360;
    if (angleValue > 360) angleValue -= 360;
  }
  angleValue = mapf(angleValue, 0., 360., 360., 0.);
  return angleValue;
}
#endif


#ifdef HAS_ANEMOMETER
float getAnemometer(void) {
  //float anemometerValue = ((float)tops ) ;
  //float anemometerValue = ((float)tops *60 * 1000.) / (millis() - anemometerMeasuringTime); //expressed in RPM
  float anemometerValue = (((float)tops * 60 * 1000.) / (millis() - anemometerMeasuringTime)) * 0.105; //expressed in km/h after calibration : https://hackaday.io/project/190577-rezodo-long-range-irrigation-and-weather-station/log/218476-calibration
  tops = 0;
  anemometerMeasuringTime = millis();
  return anemometerValue;
}
#endif

#ifdef HAS_HX711
void GetRawWeight(void) {
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);   //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);     //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);  // initialize digital pin 5 as an input.(data Out)

  unsigned long RawWeight;
  // wait for the chip to become ready
  long startTime;
  delay(5000);             //let the HX711 warm up
  AverageWeight = 0;
  for (int j = 0; j < FILTER_SAMPLES; j++) {
    startTime = millis();

    while ((digitalRead(PIN_DOUT) == HIGH) && ((millis() - startTime) < 1000)); //wait for data conversion ready

    if ((millis() - startTime) > 1000)                                          //or time out...
    {
      Serial.println("weight error");
    }
    RawWeight = 0;
    // pulse the clock pin 24 times to read the data
    for (char i = 0; i < 24; i++) {
      digitalWrite(PIN_CLOCK, HIGH);
      delayMicroseconds(2);
      RawWeight = RawWeight << 1;
      if (digitalRead(PIN_DOUT) == HIGH) RawWeight++;
      digitalWrite(PIN_CLOCK, LOW);
    }
    // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(2);
    RawWeight = RawWeight ^ 0x800000;
    digitalWrite(PIN_CLOCK, LOW);

    weightSmoothArray[j] = RawWeight;
#ifdef xxRAW_WEIGHT_DEBUG
    Serial.print("Raw weight : \t");
    Serial.println(RawWeight);
#endif
    delayMicroseconds(60);
  }
  //digitalWrite(PIN_CLOCK, HIGH);    //to enter into power saving mode
  //median filter
  boolean done;
  float temp;
  int k, top, bottom;
  done = 0;                // flag to know when we're done sorting
  while (done != 1) { // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (int j = 0; j < (FILTER_SAMPLES - 1); j++) {
      if (weightSmoothArray[j] > weightSmoothArray[j + 1]) {    // numbers are out of order - swap
        temp = weightSmoothArray[j + 1];
        weightSmoothArray [j + 1] =  weightSmoothArray[j] ;
        weightSmoothArray [j] = temp;
        done = 0;
      }
    }
  }
  // throw out top and bottom REJECT_RATIO % of samples - limit to throw out at least one from top and bottom
  bottom = max(((FILTER_SAMPLES * REJECT_RATIO)  / 100), 1);
  top = min((((FILTER_SAMPLES * (100 - REJECT_RATIO)) / 100) + 1  ), (FILTER_SAMPLES - 1)); // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  CurrentRawWeight = 0;
  for ( int j = bottom; j < top; j++) {
    CurrentRawWeight += weightSmoothArray[j];  // total remaining indices
    k++;
  }
  CurrentRawWeight = CurrentRawWeight / k;    // divide by number of samples and return the value
  //end median filter

#ifdef RAW_WEIGHT_DEBUG
  Serial.print("Raw average weight : ");
  Serial.println(CurrentRawWeight);
#endif
}
#endif

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}

float volts(float raw) { //simple linear calibration...
  //return raw * 4.23 / 2440.14 ;  // carte 10k/10k
  return raw * 4.7 / 2485.59 ;  //calib carte Deyme (12k/10k)
}

void gotoSleep() {
  digitalWrite(LED_PIN, HIGH);   // power off sensors
  digitalWrite(PWR_PIN, LOW);    //all sensors are Off
  Serial.println("Entering DeepSleep");
  pinMode(DHTPIN, INPUT);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_DOUT, INPUT);
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  if (!hasReceivedCmd) {
    Serial.println("No Cmd received... ");
  }
  if (!hasReceivedTime) {
    Serial.println("No Time received... ");
    timeToSleep = 2;                        //reset to lowest value
  }

  int mm = (int)(floor((minute() * 60 + second()) / (60 * timeToSleep)) * timeToSleep + timeToSleep);
  if (mm > 60) mm = 60;
  Serial.print ("next integer minutes " );
  Serial.println (mm);
  long tt;                                              //time to sleep
  tt = mm * 60 - (minute() * 60 + second()) - sensorsGetTime / 1000;
  display_time();
  esp_sleep_enable_timer_wakeup(tt * uS_TO_S_FACTOR);
  esp_deep_sleep_start();                               //enter deep sleep mode
  delay(1000);
  abort();
}

// TH mods start here
#ifdef HAS_MQTT
WiFiClient espClient;
PubSubClient mqttclient(espClient);

IpAddress mqttServer (192, 168, 1, 50);
const char *mqttUser = "user";
const char *mqttPass = "pass";
const char *mqttSubTopic = "subtopic";
const char *mqttPubTopic = "pubtopic";

void mqttCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == mqttSubTopic) {
    Serial.println(messageTemp);
    readJSON(messageTemp.c_str());
  }
}

void reconnect(void) {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect("SuperWeatherClient", mqttUser, mqttPass)) {
      Serial.println("connected");
      // Subscribe
      mqttclient.subscribe(mqttSubTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif

#if defined HAS_MQTT || defined HAS_LORA
bool readJSON(char *json) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, json, 256);

  if (error) {
    Serial.println("Deserialization error!");
    return false;
  }

  if (doc.containsKey("time")) {
    timeCmd[0] = doc["time"][0];
    timeCmd[1] = doc["time"][1];
    timeCmd[2] = doc["time"][2];
    timeCmd[3] = doc["time"][3];
    timeCmd[4] = doc["time"][4];
    timeCmd[5] = doc["time"][5];
  }
  if (doc.containsKey("tts")) { // timeToSleep
    timeToSleep = doc["tts"];
    hasReceivedTime = true;
  }
  if (doc.containsKey("configWiFi")) {
    configWiFi = doc["configWiFi"];
    preferences.putBool("configWiFi", configWiFi);
    hasReceivedCmd = true;
  }
  if (doc.containsKey("recv")) {
    hasSentData = doc["recv"];
  }
  return true;
}

// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}

int writeJSON(char *json) {
  DynamicJsonDocument doc(256);

  doc["H"] = round2(humidity);
  doc["T"] = round2(temperature);
  doc["P"] = round2(pressure);
  doc["wAng"] = round2(windAngle);
  doc["wSp"] = round2(windSpeed / 1.609);
  doc["wSpMax"] = round2(windSpeedMax / 1.609);
  doc["rain"] = round2(rain);
  doc["Vin"] = round2(Vin);
#ifdef HAS_LDR
  doc["LDR"] = ldr;
#endif

  serializeJson(doc, json, 256);
  return measureJson(doc); // return json size
}

void receiveLoRa() {
  String loraData;

  //try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    byte recipient = LoRa.read();
    byte sender = LoRa.read();
    byte incomingLen = LoRa.read();

    while(LoRa.available()) {
      loraData += (char)LoRa.read();
    }

    if (incomingLen != loraData.length()) {
      Serial.println("Error: Message length does not match length");
      return;
    }
    if (recipient != localAddr) {
      Serial.println("Error: Recipient address does not match local address");
      return;
    }

    char *loraJSON = &loraData[0];
    if (readJSON(loraJSON)) {
      sendStateTimeOut = 15000; //decrease the timeout value

      Serial.println("Received packet ");
      Serial.println(loraJSON);
      int rssi = LoRa.packetRssi();
      Serial.printf(" with RSSI %d\n", rssi);

      if (!timeSetInLoop) {
        setTime(timeCmd[3], timeCmd[4], timeCmd[5], timeCmd[2], timeCmd[1], timeCmd[0]);
        Serial.print("Setting time to ");
        display_time();
        Serial.printf("\nNew timeToSleep: %d\n", timeToSleep);
        Serial.printf("New configWiFi: %d\n", configWiFi);

        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        tvsec = current_time.tv_sec;
        hasRtcTime = true;

        // set DS1302 RTC time
        Ds1302::DateTime dt;
        dt.year = timeCmd[0];
        dt.month = timeCmd[1];
        dt.day = timeCmd[2];
        dt.hour = timeCmd[3];
        dt.minute = timeCmd[4];
        dt.second = timeCmd[5];
        if (rtc.isHalted()) DBG("RTC is halted...");
        rtc.setDateTime(&dt);

        timeSetInLoop = true;
      }
      else {
        Serial.println("Already set time once this loop");
      }
    }
  }
}

void sendLoRa(void) {
  char weatherOutput[256];
  int jsonLen = writeJSON(weatherOutput);

  Serial.println("Sending packet: ");
  Serial.println(weatherOutput);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.write(destAddr);
  LoRa.write(localAddr);
  LoRa.write(jsonLen);
  LoRa.print(weatherOutput);
  LoRa.endPacket();
}

void radioLoop() {
  int del = 250;
  int tries = 3;

  for (int i = 0; i < 30; i++) {
    if (hasSentData && timeSetInLoop) {
      break;
    }
    if (!hasSentData) {
      sendLoRa();
      delay(del);
    }
    if (!timeSetInLoop) {
      for (int j = 0; j < tries; j++) { // try 3 times
        receiveLoRa();
        delay(del);
      }
    }
  }
}

#ifdef HAS_MQTT
void sendMQTT() {
  mqttclient.setServer(mqtt_server, 1883);
  mqttclient.setCallback(callback);

  if (!mqttclient.connected()) {
    reconnect();
  }
  mqttclient.loop();

  const char *mqttPubString = writeJSON();

  // send it
  Serial.println("Publishing to MQTT Server: ");
  Serial.println(mqttPubString);
  mqttclient.publish("ha/weather", mqttPubString);

  delay(100);
}
#endif
#endif //defined HAS_MQTT || HAS_LORA

#ifdef HAS_LDR
void readLDR(void) {
  ldr = analogRead(LDR);
}
#endif

void loop(void) {
  const int sleepTimeOut = 45000; // time to keep weather station awake

#ifdef HAS_ANEMOMETER
  if (((millis() - windTimeOut) > 5000)) {  //compute windSpeedMax every 5s
    windTimeOut = millis();
    windSpeed = getAnemometer();
    windSpeedMax = max(windSpeedMax, windSpeed);
    Serial.print(" rot speed (km/h): ");
    Serial.print(windSpeed);
    Serial.print(" max speed (km/h): ");
    Serial.println(windSpeedMax);
    Serial.print(" rot speed (mph): ");
    Serial.print(windSpeed / 1.609);
    Serial.print(" max speed (mph): ");
    Serial.println(windSpeedMax / 1.609);
  }
#endif

  if ((((millis() - telnetTimeOut) > 3000)) && hasWifiCredentials) { //debug with telnet (Termius on Android port 23)
    telnetTimeOut = millis();
    if (touch3detected) {
      TelnetStream.println("-=[M3 Touched]=- ==> sensors calibration");
      Serial.println("-=[M3 Touched]=- ==> sensors calibration");
    }
#ifdef HAS_HX711
    TelnetStream.print("rain weight: ");
    TelnetStream.print(rainWeight);
    TelnetStream.println(" g");
#endif
#ifdef HAS_ANEMOMETER
   TelnetStream.print(" wind speed (km/h): ");
    TelnetStream.print(windSpeed);
    TelnetStream.print(" max speed (km/h): ");
    TelnetStream.println(windSpeedMax);
#endif
#ifdef HAS_AS5600
    TelnetStream.print("wind direction: ");
    TelnetStream.print(windAngle);
    TelnetStream.print("   now: ");
    TelnetStream.println( getWindAngle());
#endif
#ifdef HAS_BME280
    TelnetStream.print("temperature = ");
    TelnetStream.print(temperature);

    TelnetStream.print(" °C   pressure = ");
    TelnetStream.print( pressure);
    TelnetStream.print(" hPa   humidity = ");
    TelnetStream.print(humidity);
    TelnetStream.println(" %");
#endif
#ifdef HAS_DS18B20
    TelnetStream.print("outside temperature = ");
    TelnetStream.print(outsideTemperature);
    TelnetStream.println(" °C");
#endif
#ifdef HAS_LDR
    TelnetStream.print("LDR value: ");
    TelnetStream.println(ldr);
#endif
    TelnetStream.print("Vin = ");
    TelnetStream.print(Vin);
    TelnetStream.println( "V");
  }

  if (millis() > sleepTimeOut) {
    Serial.println ("===> Been awake too long... sending data then sleeping");
#if defined HAS_LORA
    radioLoop();
#endif
#ifdef HAS_MQTT
    sendMQTT();
#endif
    gotoSleep();
  }
}

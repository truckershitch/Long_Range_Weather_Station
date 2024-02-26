/*
See https://www.fernandok.com/2018/12/nao-perca-tempo-use-ntp.html
Also see https://github.com/hutscape/hutscape.github.io/tree/master/_tutorials/code/lora-duplex-a
and https://github.com/hutscape/hutscape.github.io/tree/master/_tutorials/code/lora-duplex-b
*/

// #define START_WIFI_MANAGER // uncomment to tell weather station to start wifi manager (can update firmware)

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define CS 18
#define RST 23
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 915E6

//OLED pins
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 0 // is this correct?
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

int timeZone = -5;
unsigned long ntpTimeout;

//Nomes dos dias da semana
char* dayOfWeekNames[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//Struct com os dados do dia e hora
struct Date{
    int dayOfWeek;
    int day;
    int month;
    int year;
    int hours;
    int minutes;
    int seconds;
};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
IPAddress mqttServer(192, 168, 1, 50);
const char *ssid = "collardgreens";
const char *passwd = "piccoloandochoarefriends";
const char *mqttUser = "homeassistant";
const char *mqttPass = "assistthisbitch";
const char *mqttSubTopic = "ha/outside/weather_station/cmd"; // commands sent by Home Assistant via MQTT
const char *mqttPubTopic = "ha/outside/weather_station/stat"; // sent to MQTT Server
const char *mqttIBTopic = "ha/outside/weather_station/ib"; // Home Assistant W.S. Input Boolean

bool pubInLoop = false; // mqtt publish once per radioLoop()

#ifdef START_WIFI_MANAGER
const int timeToSleepDefault = 2;
const bool configWiFiDefault = true;
#else
const int timeToSleepDefault = 10;
const bool configWiFiDefault = false;
#endif

int timeToSleep = timeToSleepDefault; // weather station will sleep this long if it receives the broadcast
bool configWiFi = configWiFiDefault; // weather station will start WiFiManager if true

int interval = 500;
long lastSendTime = 0;

byte localAddr = 0xAA;
byte destAddr = 0xBB;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA RECEIVER");
  display.display();

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, CS);
  //setup LoRa transceiver module
  LoRa.setPins(CS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa failed to start!");
    while (1);
  }
  Serial.println("LoRa Initialized OK!");
  display.setCursor(0,10);
  display.println("LoRa Initialized OK!");
  display.display();

  connectWiFi();
  connectMQTT();
  getNTPTime();
  radioLoop();

  ntpTimeout = millis();

  //Cria uma nova tarefa no core 0
  xTaskCreatePinnedToCore(
      wifiConnectionTask,     //Função que será executada
      "wifiConnectionTask",   //Nome da tarefa
      10000,                  //Tamanho da memória disponível (em WORDs)
      NULL,                   //Não vamos passar nenhum parametro
      2,                      //prioridade
      NULL,                   //Não precisamos de referência para a tarefa
      0);                     //Número do core
}

void connectWiFi() {
  Serial.println("Connecting WiFi");
  display.println("Connecting WiFi");

  //Troque pelo nome e senha da sua rede WiFi
  WiFi.begin(ssid, passwd);

  //Espera enquanto não estiver conectado
  while(WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      display.print(".");
      display.display();
      delay(500);
  }

  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());

  display.println();
  display.println("Connected to ");
  display.println(WiFi.SSID());
  display.display();
}

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

  if (String(topic) == mqttSubTopic) {
    Serial.println(messageTemp);
  }

  // update timeToSleep and configWiFi
  readJSON(messageTemp.c_str());
}

void connectMQTT() {
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(mqttCallback);
  checkMQTTConnection();
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT Connection...");
    if (mqttClient.connect("TTGO_Lora", mqttUser, mqttPass)) {
      Serial.println("connected");
      mqttClient.subscribe(mqttSubTopic);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(", trying again in 5 seconds");

      delay(5000);
    }
  }
}

void checkMQTTConnection() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
}

void mqttPublish(char *packet) {
    if (packet != "") {
    const char *msg = "Published to MQTT Server";
    Serial.println(msg);
    Serial.println(packet);

    mqttClient.publish(mqttPubTopic, packet);
  }
}

//Tarefa que verifica se a conexão caiu e tenta reconectar
void wifiConnectionTask(void* param) {
  while(true) {
      //Se a WiFi não está conectada
      if(WiFi.status() != WL_CONNECTED) {
          connectWiFi();
      }
      //Delay de 100 ticks
      vTaskDelay(100);
  }
}

void getNTPTime() {
  configTime(timeZone * 3600, 3600, "192.168.1.1");
}

Date getDate() {
  struct tm timeinfo;

  Date date;

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  date.year = timeinfo.tm_year + 1900;
  date.month = timeinfo.tm_mon + 1;
  date.day = timeinfo.tm_mday;
  date.hours = timeinfo.tm_hour;
  date.minutes = timeinfo.tm_min;
  date.seconds = timeinfo.tm_sec;
  date.dayOfWeek = timeinfo.tm_wday;

  return date;
}

bool readJSON(const char *json) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, json, 256);

  if (error) {
    Serial.println("Deserialization error!");
    return false;
  }

  if (doc.containsKey("tts")) {
    timeToSleep = doc["tts"];  // update timeToSleep
  }
  if (doc.containsKey("configWiFi")) {
    configWiFi = doc["configWiFi"]; // update configWiFi
  }
  if (doc.containsKey("haIBCmd")) { // weather station received command
    if (doc["haIBCmd"] == "off") {
      // send mqtt to HA - disable the input boolean
      uint8_t msg[4] = "off";
      mqttClient.publish(mqttIBTopic, msg, sizeof(msg), true);
    }
  }

  return true;
}

int writeJSON(char *json) {
  DynamicJsonDocument doc(256);

  Date date = getDate();
  char out[128];

  doc["time"][0] = date.year;
  doc["time"][1] = date.month;
  doc["time"][2] = date.day;
  doc["time"][3] = date.hours;
  doc["time"][4] = date.minutes;
  doc["time"][5] = date.seconds;

  doc["recv"] = (int)pubInLoop;
  doc["tts"] = (int)timeToSleep;
  doc["configWiFi"] = (int)configWiFi;

  serializeJson(doc, json, 128);
  return measureJson(doc); // return json length
}

void receiveLoRa() {
  String loraData;
  DynamicJsonDocument doc(256);

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
      Serial.print("Received packet ");
      Serial.println(loraJSON);
      int rssi = LoRa.packetRssi();
      Serial.printf(" with RSSI %d\n", rssi);

      if (!pubInLoop) {
        pubInLoop = true;

        mqttPublish(loraJSON);
        Serial.println();

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Published to MQTT");
        Date date = getDate();
        display.printf("%02d/%02d/%d  %02d:%02d:%02d\n",
            date.month,
            date.day,
            date.year,
            date.hours,
            date.minutes,
            date.seconds);
        display.display();
      }
      else {
        Serial.println("Already published to MQTT");
      }
    }
  }
}

void sendLoRa() {
  char wsJSON[256];
  int jsonLen = writeJSON(wsJSON);

  Serial.print("Sending packet: ");
  Serial.println(wsJSON);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.write(destAddr);
  LoRa.write(localAddr);
  LoRa.write(jsonLen);
  LoRa.print(wsJSON);
  LoRa.endPacket();
}

void radioLoop() {
  int del = 250;
  int tries = 10;
  pubInLoop = false;

  getNTPTime();
  for (int i = 0; i < tries; i++) {
    sendLoRa();
    delay(del);
    for (int j = 0; j < tries; j++) {
      receiveLoRa();
      delay(del);
    }
  }
}

void loop() {
  // timeToSleep = timeToSleepDefault;
  // configWiFi = configWiFiDefault;

  checkMQTTConnection();
  mqttClient.loop();

  if (millis() - ntpTimeout >= 5000) { // every 5 secs
    radioLoop();
    ntpTimeout = millis();
  }

  delay(100);
}

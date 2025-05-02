/*
LoRa_TTGO_Reciever_v2

For communication with Long Range Weather Station
Version 2

Created April 19, 2025
Modified April 28, 2025

See https://www.fernandok.com/2018/12/nao-perca-tempo-use-ntp.html
Also see https://github.com/hutscape/hutscape.github.io/tree/master/_tutorials/code/lora-duplex-a
and https://github.com/hutscape/hutscape.github.io/tree/master/_tutorials/code/lora-duplex-b
*/


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

#include "settings.h"

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
unsigned long radioLoopTimeout;

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

bool pubInLoop = false; // mqtt publish once per radioLoop()

// const int timeToSleepDefault = 10;
const int timeToSleepDefault = 15;
const bool configWiFiDefault = false;

int timeToSleep = timeToSleepDefault; // weather station will sleep this long if it receives the broadcast
bool configWiFi = configWiFiDefault; // weather station will start WiFiManager if true

int interval = 500;
long lastSendTime = 0;

byte localAddr = 0xAA;
byte destAddr = 0xBB;

void printlnAll(char *text) {
  //print to Serial and Display
  Serial.println(text);
  display.println(text);
}

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
  display.setCursor(0,10);
  printlnAll("LoRa Initialized OK!");
  display.display();

  connectWiFi();
  connectMQTT();
  getNTPTime();
  radioLoop();

  radioLoopTimeout = millis();

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
  printlnAll("Connecting WiFi");

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

  printlnAll("");
  char connTo[] = "Connected to ";
  printlnAll(strcat(connTo, WiFi.SSID().c_str()));

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

void mqttPublish(const char *topic, char *packet) {
    if (packet != "") {
    const char *msg = "[= Published to MQTT =] Topic";
    Serial.printf("%s %s\n%s\n\n", msg, topic, packet);

    mqttClient.publish(topic, packet);
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
  // Below is not yet implemented
  if (doc.containsKey("haIBCmd")) { // weather station received command
    if (doc["haIBCmd"] == "off") {
      // send mqtt to HA - disable the input boolean
      char *msg = "off";
      mqttPublish(mqttIBTopic, msg);
    }
  }

  return true;
}

int writeJSON(char *json, bool ack) {
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
  doc["ACK"] = (int)ack;

  serializeJson(doc, json, 128);
  return measureJson(doc); // return json length
}

void serialDate() {
  Date date = getDate();
  Serial.printf("%02d/%02d/%d  %02d:%02d:%02d\n",
      date.month,
      date.day,
      date.year,
      date.hours,
      date.minutes,
      date.seconds);
}

bool receiveLoRa() {
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
      return false;
    }
    if (recipient != localAddr) {
      Serial.println("Error: Recipient address does not match local address");
      return false;
    }

    char *loraJSON = &loraData[0];
    if (readJSON(loraJSON)) {
      char *pfx = "[= Received Packet =] ";
      // Serial.print("**Received packet ");
      // Serial.println(loraJSON);
      // int rssi = LoRa.packetRssi();
      // Serial.printf(" with RSSI %d\n", rssi);

      if (!pubInLoop) {
        pubInLoop = true;

        Serial.printf("\n%s %s\n", pfx, loraJSON);
        mqttPublish(mqttPubTopic, loraJSON);

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

        return true;
      }
      else {
        Serial.printf("%s %s\n", pfx, loraJSON);
        Serial.println("[= NOT PUBLISHED =]");

        return false;
      }
    }
  }
}

void sendLoRa(bool rcvd) {
  char wsJSON[256];
  int jsonLen = writeJSON(wsJSON, rcvd);

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
  int del = 200;
  int sndTries = 5;
  int rcvTries = 5;
  int ackTries = 30;
  bool received = false;

  getNTPTime();

  if (!received) { // we do not have the W.S. JSON so send ACK 0
    Serial.println("Sending packet with ACK 0");
    for (int i = 0; i < sndTries; i++) {
      sendLoRa(received); // send ACK 0
      delay(del);
    }
  }

  if (!received) {
    Serial.println("Listening for data");

    for (int i = 0; i < rcvTries; i++) {
      received = receiveLoRa(); // waiting for W.S. JSON
      delay(del);

      if (received) {
        Serial.println("[= Received W.S. JSON! =]");
        break;
      }
    }
  }
  
  if (received) {
    Serial.println("Sending packet with ACK 1");

    for (int i = 0; i < ackTries; i++) {
      sendLoRa(received); // send ACK 1
      delay(del * 2);
    }

    received = false; // reset boolean
  }
}

void loop() {
  // timeToSleep = timeToSleepDefault;
  // configWiFi = configWiFiDefault;

  checkMQTTConnection();
  mqttClient.loop();

  if (millis() - radioLoopTimeout >= 5000) { // every 5 secs
    radioLoop();
    radioLoopTimeout = millis();
    pubInLoop = false;
  }

  delay(100);
}

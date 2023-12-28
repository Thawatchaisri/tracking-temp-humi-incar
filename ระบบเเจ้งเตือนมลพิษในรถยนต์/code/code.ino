#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <iSYNC.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "SparkFun_SGP30_Arduino_Library.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "SimpleTimer.h"
#include <TinyGPS++.h>
#include <TridentTD_LineNotify.h>

WiFiClient client;
iSYNC iSYNC(client);

String iSYNC_USERNAME = "stanutty1123";
String iSYNC_KEY = "5fb2aa195e614c07a2d8a206";
String iSYNC_AUTH = "5fb2aa015e614c07a2d8a204";

const int LedblynkGreen = 19;
const int LedblynkRed = 18;
const int AP_Config = 5;
#define buzzer 23

LiquidCrystal_I2C lcd(0x3F, 20, 4);
SGP30 mySensor;
SoftwareSerial serial_connection(16, 17);
TinyGPSPlus gps;
SimpleTimer timer;

char line_token1[45] = "";
char line_token2[45] = "";
char line_token3[45] = "";

float ppm;
float RS_gasc = 0;
float ratioc = 0;
float sensorValuec = 0;
float sensor_voltc = 0;
float R0c = 7200.0;
float xc = 0;

float sensor_volt;
float RS_gas;
float R0 = 15000;
float ratio;
float NUTLPG_PPM;
float x = 0;

bool shouldSaveConfig = false;

void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  Serial.print("[iSYNC]-> ");
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (msg.startsWith("LINE:")) msg = msg.substring(5);

  if (msg.equals("co")) {
    iSYNC.mqPub(iSYNC_KEY, "CO ตอนนี้มีปริมาณ " + String(ppm) + " ppm ถ้ายังไม่ถึง 30-35ppm ถือว่ายังปลอดภัยครับ"); // Publish
  } else if (msg.equals("lpg")) {
    iSYNC.mqPub(iSYNC_KEY, "LPG ตอนนี้มีปริมาณ " + String(NUTLPG_PPM) + " ppb ครับ ถ้ายังไม่ถึง 500ppb ถือว่ายังปลอดภัยครับ"); // Publish
  } else if (msg.equals("co2")) {
    iSYNC.mqPub(iSYNC_KEY, "Co2 ตอนนี้มีปริมาณ " + String(mySensor.CO2) + " ppm ครับ ถ้ายังไม่ถึง 1500-1800ppm ถือว่ายังปลอดภัยครับ"); // Publish
  } else if (msg.equals("tvoc")) {
    iSYNC.mqPub(iSYNC_KEY, "TVOCS ตอนนี้มีปริมาณ " + String(mySensor.TVOC) + " ppb ครับ"); // Publish
  } else if (msg.equals("พิกัด")) {
    iSYNC.mqPub(iSYNC_KEY, "ตอนนี้คุณอยู่ที่ " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6)); // Publish
  }
}

void connectMQTT() {
  while (!iSYNC.mqConnect()) {
    Serial.println("Reconnect MQTT...");
    delay(3000);
  }
  iSYNC.mqPub(iSYNC_KEY, "พร้อมรับคำสั่งแล้วจ๊ะพี่จ๋า"); // Publish on Connect
  iSYNC.mqSub(iSYNC_KEY); // Subscribe key
}

void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(LedblynkGreen, OUTPUT);
  pinMode(LedblynkRed, OUTPUT);
  pinMode(AP_Config, INPUT_PULLUP);

  digitalWrite(LedblynkRed, LOW);
  digitalWrite(LedblynkGreen, LOW);
  digitalWrite(buzzer, HIGH);

  Serial.begin(115200);

  timer.setInterval(3000, NutreadSensor);
  timer.setInterval(360000, toline);
  timer.setInterval(1000, recon);
  timer.setInterval(1000, calco);
  timer.setInterval(1000, callpg);

  lcd.begin();
  lcd.backlight();
  Wire.begin();
  serial_connection.begin(9600);

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(line_token1, json["line_token1"]);
          strcpy(line_token2, json["line_token2"]);
          strcpy(line_token3, json["line_token3"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter custom_text0("<p> </p>");
  WiFiManagerParameter custom_text1("<label>1:  ID Line 1 </label>");
  WiFiManagerParameter custom_text2("<label>2:  ID Line 2 </label>");
  WiFiManagerParameter custom_text3("<label>3:  ID Line 3 </label>");

  WiFiManagerParameter custom_line_token1("line_token1", "line_token1", line_token1, 45);
  WiFiManagerParameter custom_line_token2("line_token2", "line_token2", line_token2, 45);
  WiFiManagerParameter custom_line_token3("line_token3", "line_token3", line_token3, 45);

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_text0);
  wifiManager.addParameter(&custom_text1);
  wifiManager.addParameter(&custom_line_token1);
  wifiManager.addParameter(&custom_text2);
  wifiManager.addParameter(&custom_line_token2);
  wifiManager.addParameter(&custom_text3);
  wifiManager.addParameter(&custom_line_token3);

  wifiManager.autoConnect("AutoConnectAP");

  strcpy(line_token1, custom_line_token1.getValue());
  strcpy(line_token2, custom_line_token2.getValue());
  strcpy(line_token3, custom_line_token3.getValue());

  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["line_token1"] = line_token1;
    json["line_token2"] = line_token2;
    json["line_token3"] = line_token3;

    File configFile = SPIFFS.open("/config.json", "w");
    if (configFile) {
      json.printTo(configFile);
      configFile.close();
    } else {
      Serial.println("failed to open config file for writing");
    }
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  digitalWrite(LedblynkRed, HIGH);
  digitalWrite(LedblynkGreen, HIGH);
  digitalWrite(buzzer, LOW);

  Serial.println("Connect to MQTT");
  connectMQTT();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connect to MQTT");
  lcd.setCursor(0, 1);
  lcd.print("IP:" + WiFi.localIP());
}

void loop() {
  iSYNC.mqLoop();
  timer.run();
}

void recon() {
  if (digitalRead(AP_Config) == LOW) {
    WiFi.disconnect(true);
    delay(1000);
    Serial.println("Reconfig");
    digitalWrite(LedblynkGreen, LOW);
    digitalWrite(LedblynkRed, HIGH);
    digitalWrite(buzzer, HIGH);
    ESP.restart();
  }
}

void toline() {
  iSYNC.mqPub(iSYNC_KEY, "รีบุ๊คตัวเองทันที " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6)); // Publish
}

void calco() {
  if (isnan(ppm)) {
    lcd.setCursor(0, 0);
    lcd.print("CO Err");
  } else if (ppm <= 30) {
    lcd.setCursor(0, 0);
    lcd.print("CO ปลอดภัย ปริมาณ " + String(ppm) + " ppm ");
  } else if (ppm > 30) {
    lcd.setCursor(0, 0);
    lcd.print("CO แจ้งเตือน ปริมาณ " + String(ppm) + " ppm ");
    tone(buzzer, 4000, 500);
  }
}

void callpg() {
  if (isnan(NUTLPG_PPM)) {
    lcd.setCursor(0, 1);
    lcd.print("LPG Err");
  } else if (NUTLPG_PPM <= 500) {
    lcd.setCursor(0, 1);
    lcd.print("LPG ปลอดภัย ปริมาณ " + String(NUTLPG_PPM) + " ppb ");
  } else if (NUTLPG_PPM > 500) {
    lcd.setCursor(0, 1);
    lcd.print("LPG แจ้งเตือน ปริมาณ " + String(NUTLPG_PPM) + " ppb ");
    tone(buzzer, 4000, 500);
  }
}

void NutreadSensor() {
  NutreadSensor_Serial();

  ppm = mySensor.getCO2();

  if (isnan(ppm)) {
    lcd.setCursor(0, 2);
    lcd.print("CO2 Err");
  } else if (ppm <= 1500) {
    lcd.setCursor(0, 2);
    lcd.print("CO2 ปลอดภัย ปริมาณ " + String(ppm) + " ppm ");
  } else if (ppm > 1500) {
    lcd.setCursor(0, 2);
    lcd.print("CO2 แจ้งเตือน ปริมาณ " + String(ppm) + " ppm ");
    tone(buzzer, 4000, 500);
  }
}

void NutreadSensor_Serial() {
  if (mySensor.begin(serial_connection)) {
    Serial.println("Waiting for SGP30 to generate baselines. Please wait 10 minutes...");
    mySensor.initAirQuality();
    delay(10000);
    Serial.println("Baselines are ready.");
  } else {
    Serial.println("SGP30 not detected.");
  }
}

void NutreadSensor_I2C() {
  if (mySensor.begin(Wire)) {
    Serial.println("Waiting for SGP30 to generate baselines. Please wait 10 minutes...");
    mySensor.initAirQuality();
    delay(10000);
    Serial.println("Baselines are ready.");
  } else {
    Serial.println("SGP30 not detected.");
  }
}

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
#include "sensorData.ino"
#include "controller.ino"
#include "display.ino"

WiFiClient client;
iSYNC iSYNC(client);
SensorData sensorData;

// ... (rest of your code)

void setup() {
  // ... (existing setup code)

  // Initialize components
  Display::init();
  Controller::connectMQTT();
  Controller::initAirQualitySensor();
}

void loop() {
  iSYNC.mqLoop();
  Controller::readSensorData(sensorData);
  Controller::checkAndAlert(sensorData);
  Display::showStatus("Running");
  delay(1000);
}

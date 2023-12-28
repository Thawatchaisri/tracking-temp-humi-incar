// controller.h
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sensorData.ino"

class Controller {
public:
  static void handleMQTTMessage(const char* topic, byte* payload, unsigned int length, SensorData& sensorData);
  static void connectMQTT();
  static void reconnectIfAPConfig();
  static void publishToLine(const char* message, SensorData& sensorData);
  static void checkAndAlert(SensorData& sensorData);
  static void readSensorData(SensorData& sensorData);
  static void initAirQualitySensor();
};

#endif // CONTROLLER_H

// sensorData.h
#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

class SensorData {
public:
  float ppm;
  float NUTLPG_PPM;
  float co2;

  void update(float ppmValue, float lpgValue, float co2Value) {
    ppm = ppmValue;
    NUTLPG_PPM = lpgValue;
    co2 = co2Value;
  }
};

#endif // SENSOR_DATA_H

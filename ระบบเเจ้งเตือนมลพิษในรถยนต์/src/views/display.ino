// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include "sensorData.ino"

class Display {
public:
  static void init();
  static void showStatus(const char* status);
  static void showCOStatus(float ppm);
  static void showLPGStatus(float NUTLPG_PPM);
  static void showCO2Status(float co2);
};

#endif // DISPLAY_H

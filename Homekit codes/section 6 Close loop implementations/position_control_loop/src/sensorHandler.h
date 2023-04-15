#ifndef SENSORHANDLER_H
#define SENSORHANDLER_H
#include <Arduino.h>
#include "mapper/mapper.h"
#include "motor/motor.h"
using namespace std;

class SensorHandler
{
public:
  static const int sensorMinValue = 0;
  static const int sensorMaxValue = 1023;
  static const int speendMinValue = 0;
  static const int speedMaxValue = 180;
  int scaleSize = sensorMaxValue - sensorMinValue;
  int middleStartThreshole, middleEndThreshole;
  Map *SpeedScaler;
  byte sensorPin;
  SensorHandler(byte sensor_pin, float middleThreshole);
  int get_speed();
  int get_state();
};

#endif
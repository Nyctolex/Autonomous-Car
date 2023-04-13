#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <Arduino.h>
#include "mapper/mapper.h"
using namespace std;

class PID_Contrller{
  public:
  double Kp;
  double Ki;
  double Kd;
  double target_value;
  double target_min_value, target_max_value, target_value_range;
  float cumulative_error; // units are cm, this is needed for I-term in PID 
  double inetgrated_error;
double previous_error; // this is for D term in PID
double dt;
double prev_time;
Map *mapper;
PID_Contrller(double kp, double ki, double kd, double target_value, double target_min_value, double target_max_value);
void reset_controller();
double next(double sensor_output);

};

#endif
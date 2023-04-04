#ifndef MAPPER_H
#define MAPPER_H
#include <Arduino.h>
class Map
{
  // A class that linearly maps between input range of values to output range of values
public:
  float input_min_value;
  float input_max_value;
  float Output_min_value;
  float output_max_value;
  float input_scale_size;
  float output_scale_size;
  Map(float input_min_value, float input_max_value, float output_min_value, float output_max_value);
  float map_value(float input_value);
};
#endif
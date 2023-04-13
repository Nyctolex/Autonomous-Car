#ifndef MAPPER_H
#define MAPPER_H
#include <Arduino.h>
class Map
{
  // A class that linearly maps between input range of values to output range of values
public:
  int input_min_value;
  int input_max_value;
  int Output_min_value;
  int output_max_value;
  int input_scale_size;
  int output_scale_size;
  Map(int input_min_value, int input_max_value, int output_min_value, int output_max_value);
  int map_value(int input_value);
};
#endif
#include "mapper.h"
  // A class that linearly maps between input range of values to output range of values
  Map::Map(float input_min_value, float input_max_value, float output_min_value, float output_max_value)
  {
    input_min_value = input_min_value;
    input_max_value = input_max_value;
    Output_min_value = output_min_value;
    output_max_value = output_max_value;
    input_scale_size = input_max_value - input_min_value;
    output_scale_size = output_max_value - Output_min_value;
  }
  float Map::map_value(float input_value)
  {
    // remap the analog value to a value between 0 to 1
    float zero_to_one_scale = (input_value - input_min_value) / input_scale_size;
    float voltage_value = zero_to_one_scale * output_scale_size + input_min_value;
    return voltage_value;
  }
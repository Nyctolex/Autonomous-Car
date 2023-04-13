#include "mapper.h"
  Map::Map(int input_min_value, int input_max_value, int output_min_value, int output_max_value)
  {
    this->input_min_value = input_min_value;
    this->input_max_value = input_max_value;
    this->Output_min_value = output_min_value;
    this->output_max_value = output_max_value;
    this->input_scale_size = input_max_value - input_min_value;
    this->output_scale_size = output_max_value - Output_min_value;
  }
  int Map::map_value(int input_value)
  {
    // remap the analog value to a value between 0 to 1
    float zero_to_one_scale = (float)(input_value - this->input_min_value) / (float)this->input_scale_size;
    float voltage_value = zero_to_one_scale * (float)this->output_scale_size + (float)this->input_min_value;
    return (int)voltage_value;
  }
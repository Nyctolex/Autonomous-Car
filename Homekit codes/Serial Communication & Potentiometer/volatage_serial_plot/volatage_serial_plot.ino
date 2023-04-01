class Map
{
  // A class that linearly maps between input range of values to output range of values
public:
  float input_min_value;
  float input_max_value;
  float output_min_value;
  float output_max_value;
  float input_scale_size;
  float output_scale_size;
  Map(float input_min_value, float input_max_value, float output_min_value, float output_max_value)
  {
    this->input_min_value = input_min_value;
    this->input_max_value = input_max_value;
    this->output_min_value = output_min_value;
    this->output_max_value = output_max_value;
    this->input_scale_size = input_max_value - input_min_value;
    this->output_scale_size = output_max_value - output_min_value;
  }
  float map_value(float input_value)
  {
    // remap the analog value to a value between 0 to 1
    float zero_to_one_scale = (input_value - this->input_min_value) / this->input_scale_size;
    float voltage_value = zero_to_one_scale * this->output_scale_size + this->input_min_value;
    return voltage_value;
  }
};


Map *voltage_mapper;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // create a new mapper that maps the value between [0 1024] to [0,5]
  voltage_mapper = new Map(0, 1024, 0, 5);
}

// the loop routine runs over and over again forever:
void loop()
{
  Serial.print("Voltage:");
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  float voltage_value = voltage_mapper->map_value((float)sensorValue);
  // print out the value you read in voltages:
  Serial.println(voltage_value);
  delay(100); // delay in between reads for stability
}

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
  Map(float input_min_value, float input_max_value, float output_min_value, float output_max_value)
  {
    input_min_value = input_min_value;
    input_max_value = input_max_value;
    Output_min_value = output_min_value;
    output_max_value = output_max_value;
    input_scale_size = input_max_value - input_min_value;
    output_scale_size = output_max_value - Output_min_value;
  }
  float map_value(float input_value)
  {
    // remap the analog value to a value between 0 to 1
    float zero_to_one_scale = (input_value - input_min_value) / input_scale_size;
    float voltage_value = zero_to_one_scale * output_scale_size + input_min_value;
    return voltage_value;
  }
};

Map *brighness_mapper;
const int led = 9;
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  brighness_mapper = new Map(0,1024, 0, 255);
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop()
{
  
  // read the input on analog pin 0:
  int sensor_value = analogRead(A0);
  // convert into the led scale
  int brightness = (int) brighness_mapper->map_value((float)sensor_value);
  analogWrite(led, brightness);
  // print out the value you read in voltages:
  Serial.print("Brightness_value:");
  Serial.println(brightness);
  delay(100); // delay in between reads for stability
}

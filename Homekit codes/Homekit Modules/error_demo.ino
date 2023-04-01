#define RED_LED_PIN 9
#define BLUE_LED_PIN 10
#define GREEN_LED_PIN 11
#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 6

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

Map *mapper;
void setup()
{
  Serial.begin(9600);
  mapper = new Map(0,1024,0,255);
  digitalWrite(COUNTERCLOCKWISE_PIN, LOW);
  digitalWrite(CLOCKWISE_PIN, LOW);
  // COUNTERCLOCKWISE_PIN
  //CLOCKWISE_PIN
}

// the loop routine runs over and over again forever:
void loop()
{
  int sensor_value = analogRead(A0);
  int speed = (int)mapper->map_value( (float)sensor_value );
  
  analogWrite(CLOCKWISE_PIN, speed);
  analogWrite(COUNTERCLOCKWISE_PIN, 0);
  delay(10);
}

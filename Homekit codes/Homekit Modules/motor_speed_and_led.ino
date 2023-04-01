#define RED_LED_PIN 9
#define BLUE_LED_PIN 11
#define GREEN_LED_PIN 10
#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 4

// the diffrent state the motor could be in
enum State
{
  forward,
  backward,
  coast,
  halt
};

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

class Motor
{
    // Handle the motor speed and state
public:
  int clockwise_pin, counter_clockwise_pin;
  Motor(int clockwise_pin, int counter_clockwise_pin)
  {
    this->clockwise_pin = clockwise_pin;
    this->counter_clockwise_pin = counter_clockwise_pin;
    pinMode(this->clockwise_pin, OUTPUT);
    pinMode(this->counter_clockwise_pin, OUTPUT);
  }

  void update(int state, int speed)
  {
    switch (state)
    {
    case State::forward:
      digitalWrite(this->counter_clockwise_pin, LOW);
      analogWrite(this->clockwise_pin, speed);
      break;
    case State::backward:
    
      digitalWrite(this->clockwise_pin, LOW);
      analogWrite(this->counter_clockwise_pin, speed);
      break;
    case State::halt:
digitalWrite(this->counter_clockwise_pin, HIGH);
digitalWrite(this->clockwise_pin, HIGH);
    break;
    default:
    // the default is coast
      digitalWrite(this->counter_clockwise_pin, LOW);
      digitalWrite(this->clockwise_pin, LOW);
    }
  }
};

class SensorHandler
{
public:
  static const int sensorMinValue = 0;
  static const int sensorMaxValue = 1023;
  static const int speendMinValue = 0;
  static const int speedMaxValue = 255;
  int scaleSize = sensorMaxValue - sensorMinValue;
  int MiddleStartThreshole, MiddleEndThreshole;
  Map *SpeedScaler;
  byte SensorPin;
  SensorHandler(byte sensor_pin, float middleThreshole)
  {
    float middleValue = (sensorMinValue + sensorMaxValue) / 2;
    MiddleStartThreshole = (int)((float)middleValue - (float)scaleSize * middleThreshole / 2);
    MiddleEndThreshole = (int)((float)middleValue + (float)scaleSize * middleThreshole / 2);
    int Sensor_pin = sensor_pin;
    SpeedScaler = new Map(sensorMinValue, MiddleStartThreshole, speendMinValue, speedMaxValue);
  }
  int get_speed(){
    int sensorValue = analogRead(A0);
    int unscaled_speed;
    if (sensorValue < MiddleStartThreshole){
      unscaled_speed = MiddleStartThreshole - sensorValue;
    } else if (sensorValue > MiddleEndThreshole){
      unscaled_speed = sensorValue - MiddleEndThreshole;
    } else {
      unscaled_speed = 0;
    }
    int scale_speed = (int)SpeedScaler->map_value((float)unscaled_speed);

    if (Serial.availableForWrite() > 0){
  Serial.print(",");
  Serial.println(unscaled_speed);
}
    return scale_speed;
  }
  int get_state()
  {
    int sensorValue = analogRead(A0);

if (Serial.availableForWrite() > 0){
}
    if (sensorValue < MiddleStartThreshole)
    {
      return State::forward;
    }
    else if (sensorValue > MiddleEndThreshole)
    {
      return State::backward;
    }
    else
    {
      return State::coast;
    }

  }
};

class LedHandler
{
public:
  int Forward_led, Backward_led, Stop_led;
  LedHandler(int forward_led, int backward_led, int stop_led)
  {
    Forward_led = forward_led;
    Backward_led = backward_led;
    Stop_led = stop_led;
    pinMode(Forward_led, OUTPUT);
    pinMode(Backward_led, OUTPUT);
    pinMode(Stop_led, OUTPUT);
  }

  void update(int state)
  {
    switch (state)
    {
    case State::forward:
      digitalWrite(Forward_led, HIGH);
      digitalWrite(Backward_led, LOW);
      digitalWrite(Stop_led, LOW);
      break;
    case State::backward:
      digitalWrite(Forward_led, LOW);
      digitalWrite(Backward_led, HIGH);
      digitalWrite(Stop_led, LOW);
      break;
    default:
      digitalWrite(Forward_led, LOW);
      digitalWrite(Backward_led, LOW);
      digitalWrite(Stop_led, HIGH);
    }
  }
};


Motor *motor;
LedHandler *leds;
SensorHandler *sensor;
void setup()
{
  Serial.begin(9600);
  motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
  motor->update(State::coast, 0);
  leds = new LedHandler(BLUE_LED_PIN, RED_LED_PIN,GREEN_LED_PIN );
  sensor = new SensorHandler(A0, 0.3);
}

// the loop routine runs over and over again forever:
void loop()
{
int speed = sensor->get_speed();
  int state = sensor->get_state();
  motor->update(state, speed);
  leds->update(state);
  delay(100); // delay in between reads for stability
}
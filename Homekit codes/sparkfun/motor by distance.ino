#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 4

class PlotWithLabels
{
public:
    String *Labels;
    int NumLabels;
    PlotWithLabels(String *labels, int num_labels)
    {
        Labels = labels;
        NumLabels = num_labels;
    }
    void plot(int *values)
    {
        for (int i = 0; i < NumLabels; i++)
        {
            Serial.print(Labels[i]);
            Serial.print(":");
            Serial.print(values[i]);
            Serial.print("\t");
        }
        Serial.println();
    }
};

class DistanceSensorHandler
{
public:
    SFEVL53L1X distanceSensor;
    DistanceSensor()
    {
        Serial.println("VL53L1X Qwiic Test");

        if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
        {
            Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
            while (1)
                ;
        }
        Serial.println("Sensor online!");
    }
    int read()
    {
        distanceSensor.startRanging(); // Write configuration bytes to initiate measurement
        while (!distanceSensor.checkForDataReady())
        {
            delay(1);
        }
        int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();
        return distance;
    }
};

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
  static const int sensorMaxValue = 2500;
  static const int speendMinValue = 0;
  static const int speedMaxValue = 255;
  Map *SpeedScaler;
  DistanceSensorHandler *DistanceSensorHandlerObject ;
  SensorHandler(DistanceSensorHandler* distance_sensor_handler)
  {
    SpeedScaler = new Map(sensorMinValue, sensorMaxValue, speendMinValue, speedMaxValue);
    DistanceSensorHandlerObject  = distance_sensor_handler;
  }

  int get_speed(){
    int sensor_value = DistanceSensorHandlerObject->read();
    int speed = (int)SpeedScaler->map_value((float)sensor_value);
    return speed;
  }
};


#define NUM_LABELS 1
String LABELS[NUM_LABELS] = {"Speed"};
PlotWithLabels *plotter;
DistanceSensorHandler *sensor;
SensorHandler *sensor_handler;
Motor *motor;

void setup(void)
{
    Wire.begin();
    Serial.begin(115200);
    sensor = new DistanceSensorHandler();
    sensor_handler = new SensorHandler(sensor);
    plotter = new PlotWithLabels(LABELS, NUM_LABELS);
    motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
}

void loop(void)
{
    int speed = sensor_handler->get_speed();
    int values[] = {speed};
    plotter->plot(values);
    motor->update(State::forward, speed);
    delay(1);
}

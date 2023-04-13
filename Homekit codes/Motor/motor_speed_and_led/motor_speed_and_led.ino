#define RED_LED_PIN 9
#define BLUE_LED_PIN 10
#define GREEN_LED_PIN 11
#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 6
#include "src/motor/motor.h"
#include "src/motor/ledHandler.h"
#include "src/mapper/mapper.h"
using namespace std;

class SensorHandler
{
public:
  static const int sensorMinValue = 0;
  static const int sensorMaxValue = 1023;
  static const int speendMinValue = 0;
  static const int speedMaxValue = 255;
  int scaleSize = sensorMaxValue - sensorMinValue;
  int middleStartThreshole, middleEndThreshole;
  Map *speedScaler;
  byte SensorPin;
  SensorHandler(byte sensor_pin, float middleThreshole)
  {
    float middleValue = (sensorMinValue + sensorMaxValue) / 2;
    middleStartThreshole = (int)((float)middleValue - (float)this->scaleSize * middleThreshole / 2);
    middleEndThreshole = (int)((float)middleValue + (float)this->scaleSize * middleThreshole / 2);
    int maxValidScaleValue = max(middleStartThreshole, this->sensorMaxValue - middleEndThreshole);
    int Sensor_pin = sensor_pin;
    this->speedScaler = new Map(sensorMinValue, maxValidScaleValue, speendMinValue, speedMaxValue);
  }
  int get_speed()
  {
    int sensorValue = analogRead(A0);
    int unscaled_speed;
    if (sensorValue < this->middleStartThreshole)
    {
      unscaled_speed = this->middleStartThreshole - sensorValue;
    }
    else if (sensorValue > this->middleEndThreshole)
    {
      unscaled_speed = sensorValue - this->middleEndThreshole;
    }
    else
    {
      unscaled_speed = 0;
    }
    int scale_speed = this->speedScaler->map_value(unscaled_speed);
    return scale_speed;
  }

  int get_state()
  {
    int sensorValue = analogRead(A0);

    if (sensorValue < middleStartThreshole)
    {
      return State::forward;
    }
    else if (sensorValue > middleEndThreshole)
    {
      return State::backward;
    }
    else
    {
      return State::coast;
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
  leds = new LedHandler(BLUE_LED_PIN, RED_LED_PIN, GREEN_LED_PIN);
  sensor = new SensorHandler(A0, 0.3);
}

void loop()
{
  int speed = sensor->get_speed();
  int state = sensor->get_state();
  motor->update(state, speed);
  leds->update(state);
  delay(100); // delay in between reads for stability
}
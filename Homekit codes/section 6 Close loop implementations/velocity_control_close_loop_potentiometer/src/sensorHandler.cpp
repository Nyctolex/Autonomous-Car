#include "sensorHandler.h"
  SensorHandler::SensorHandler(byte sensor_pin, float middleThreshole)
  {
    float middleValue = (sensorMinValue + sensorMaxValue) / 2;
    middleStartThreshole = (int)((float)middleValue - (float)scaleSize * middleThreshole / 2);
    middleEndThreshole = (int)((float)middleValue + (float)scaleSize * middleThreshole / 2);
    int maxValidScaleValue = max(middleStartThreshole, this->sensorMaxValue - middleEndThreshole);
    this->sensorPin = sensor_pin;
    SpeedScaler = new Map(sensorMinValue, maxValidScaleValue, speendMinValue, speedMaxValue);
  }
  int SensorHandler::get_speed(){
    int sensorValue = analogRead(this->sensorPin);
    int unscaled_speed;
    if (sensorValue < middleStartThreshole){
      unscaled_speed = sensorValue - middleStartThreshole;
    } else if (sensorValue > middleEndThreshole){
      unscaled_speed = sensorValue - middleEndThreshole;
    } else {
      unscaled_speed = 0;
    }

    int scale_speed = SpeedScaler->map_value(unscaled_speed);
    return scale_speed;
  }
  int SensorHandler::get_state()
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
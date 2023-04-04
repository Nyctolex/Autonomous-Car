#include "motor.h"



  Motor::Motor(int clockwise_pin, int counter_clockwise_pin)
  {
    this->clockwise_pin = clockwise_pin;
    this->counter_clockwise_pin = counter_clockwise_pin;
    pinMode(this->clockwise_pin, OUTPUT);
    pinMode(this->counter_clockwise_pin, OUTPUT);
  }

  void Motor::update(int state, int speed)
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
#include "motor.h"

  int Motor::reverse_state(int state){
    if (state == State::forward)
      return State::backward;
    else if (state == State::backward)
      return State::forward;
    else if (state == State::halt)
      return State::coast;
    return State::halt;
  }

  Motor::Motor(int clockwise_pin, int counter_clockwise_pin)
  {
    this->clockwise_pin = clockwise_pin;
    this->counter_clockwise_pin = counter_clockwise_pin;
    pinMode(this->clockwise_pin, OUTPUT);
    pinMode(this->counter_clockwise_pin, OUTPUT);
  }

  void Motor::update(int state, int speed)
  {
    // if the speed is negative, reverse direction
    if (speed < 0){
      state = Motor::reverse_state(state);
      speed = -1*speed;
    }
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
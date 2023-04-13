#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
using namespace std;
class Motor
{
    // Handle the motor speed and state
public:
  int clockwise_pin, counter_clockwise_pin;
  Motor(int clockwise_pin, int counter_clockwise_pin);
  static int reverse_state(int state);
  void update(int state, int speed);
};

enum State
{
  forward,
  backward,
  coast,
  halt
};

#endif
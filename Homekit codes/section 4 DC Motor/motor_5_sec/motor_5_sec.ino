#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 6

#include "src/motor/motor.h"
Motor *motor;
int speed =255;
int state = State::forward;
void setup()
{
  Serial.begin(9600);
  motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
  motor->update(State::coast, 0);
}

// the loop routine runs over and over again forever:
void loop()
{
  state = Motor::reverse_state(state);
  Serial.println(state);
  motor->update(state, speed);
  delay(5000);
  motor->update(State::halt, speed);
  delay(1000);
  
}
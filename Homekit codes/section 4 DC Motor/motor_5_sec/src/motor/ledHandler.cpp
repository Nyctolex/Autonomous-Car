#include "ledHandler.h"
#include "motor.h"


LedHandler::LedHandler(int forward_led, int backward_led, int halt_led)
{
    this->forward_led = forward_led;
    this->backward_led = backward_led;
    this->halt_led = halt_led;
    pinMode(forward_led, OUTPUT);
    pinMode(backward_led, OUTPUT);
    pinMode(halt_led, OUTPUT);
}

void LedHandler::update(int state)
{
    switch (state)
    {
    case State::forward:
        digitalWrite(this->forward_led, HIGH);
        digitalWrite(this->backward_led, LOW);
        digitalWrite(this->halt_led, LOW);
        break;
    case State::backward:
        digitalWrite(this->forward_led, LOW);
        digitalWrite(this->backward_led, HIGH);
        digitalWrite(this->halt_led, LOW);
        break;
    default:
        digitalWrite(this->forward_led, LOW);
        digitalWrite(this->backward_led, LOW);
        digitalWrite(this->halt_led, HIGH);
    }
}

#include "ledHandler.h"
#include "motor.h"


LedHandler::LedHandler(int forward_led, int backward_led, int stop_led)
{
    forward_led = forward_led;
    backward_led = backward_led;
    halt_led = stop_led;
    pinMode(forward_led, OUTPUT);
    pinMode(backward_led, OUTPUT);
    pinMode(halt_led, OUTPUT);
}

void LedHandler::update(int state)
{
    switch (state)
    {
    case State::forward:
        digitalWrite(forward_led, HIGH);
        digitalWrite(backward_led, LOW);
        digitalWrite(halt_led, LOW);
        break;
    case State::backward:
        digitalWrite(forward_led, LOW);
        digitalWrite(backward_led, HIGH);
        digitalWrite(halt_led, LOW);
        break;
    default:
        digitalWrite(forward_led, LOW);
        digitalWrite(backward_led, LOW);
        digitalWrite(halt_led, HIGH);
    }
}

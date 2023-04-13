#ifndef LED_HANDLER_H
#define LED_HANDLER_H 

class LedHandler
{
public:
  int forward_led, backward_led, halt_led;
  LedHandler(int forward_led, int backward_led, int stop_led);
  void update(int state);
};
#endif
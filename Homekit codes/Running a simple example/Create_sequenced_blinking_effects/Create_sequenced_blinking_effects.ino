const int red_led = 9;
const int green_led = 10;
const int blue_led = 11;
const int blink_time = 500;

void blink_led(int led_pin, int blink_time){
  digitalWrite(led_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(blink_time);                      // wait for a second
  digitalWrite(led_pin, LOW);   // turn the LED off by making the voltage LOW
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  blink_led(red_led, blink_time);
  blink_led(green_led, blink_time);
  blink_led(blue_led, blink_time);
  blink_led(green_led, blink_time);
  delay(blink_time);
  blink_led(green_led, blink_time);
  delay(blink_time);
  blink_led(green_led, blink_time);
}

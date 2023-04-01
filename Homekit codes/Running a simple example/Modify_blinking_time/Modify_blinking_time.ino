int red_led = 9;
int green_led = 10;
int blue_led = 11;
int chosen_led = blue_led;
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(chosen_led, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(chosen_led, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(chosen_led, LOW);   // turn the LED off by making the voltage LOW
  delay(100);                      // wait for a second
}

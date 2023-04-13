// Implement a simple parser which receives an ascii string of numbers terminated with ‘\n’ converts the
//  string to a number and updates the LED's brightness accordingly.

#define RED_LED_PIN 9
#define BLUE_LED_PIN 11
#define GREEN_LED_PIN 10
int chosen_led = BLUE_LED_PIN;

void setup()
{
    Serial.begin(9600);
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(chosen_led, OUTPUT);
}

// the loop function runs over and over again forever
void loop()
{
    if (Serial.available() > 0)
    {
        int brightness = Serial.readString().toInt();  
        analogWrite(chosen_led, brightness);
    }
    delay(1);
}

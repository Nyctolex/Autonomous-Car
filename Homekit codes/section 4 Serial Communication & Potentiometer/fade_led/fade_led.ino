#include "src/mapper/mapper.h"

Map *brighness_mapper;
const int led = 9;
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  brighness_mapper = new Map(0,1024, 0, 255);
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop()
{
  
  // read the input on analog pin 0:
  int sensor_value = analogRead(A0);
  // convert into the led scale
  int brightness = (int) brighness_mapper->map_value((float)sensor_value);
  analogWrite(led, brightness);
  // print out the value you read in voltages:
  Serial.print("Brightness_value:");
  Serial.println(brightness);
  delay(100); // delay in between reads for stability
}

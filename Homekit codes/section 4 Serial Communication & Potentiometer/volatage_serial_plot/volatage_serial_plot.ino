#include "src/mapper/mapper.h"

Map *voltage_mapper;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // create a new mapper that maps the value between [0 1024] to [0,5]
  voltage_mapper = new Map(0, 1023, 0, 5);
}

// the loop routine runs over and over again forever:
void loop()
{
  Serial.print("Voltage:");
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  int voltage_value = voltage_mapper->map_value(sensorValue);
  // print out the value you read in voltages:
  Serial.println(voltage_value);
  delay(100); // delay in between reads for stability
}

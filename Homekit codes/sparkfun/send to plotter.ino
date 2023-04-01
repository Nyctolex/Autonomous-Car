/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
// Uncomment the following line to use the optional shutdown and interrupt pins.
// SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

class PlotWithLabels
{
public:
    String *Labels;
    int NumLabels;
    PlotWithLabels(String *labels, int num_labels)
    {
        Labels = labels;
        NumLabels = num_labels;
    }
    void plot(int *values)
    {
        for (int i = 0; i < NumLabels; i++)
        {
            Serial.print(Labels[i]);
            Serial.print(":");
            Serial.print(values[i]);
            Serial.print("\t");
        }
        Serial.println();
    }
};

#define NUM_LABELS 2
String LABELS[NUM_LABELS] = {"Distance(mm)", "Distance(inch)"};
PlotWithLabels *plotter;

void setup(void)
{
    Wire.begin();
    Serial.begin(115200);
    Serial.println("VL53L1X Qwiic Test");

    if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
    {
        Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (1)
            ;
    }
    Serial.println("Sensor online!");

    plotter = new PlotWithLabels(LABELS, NUM_LABELS);
}

void loop(void)
{
    distanceSensor.startRanging(); // Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
        delay(1);
    }
    int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;

    int values[] = {distance, distanceInches};
    plotter->plot(values);
}

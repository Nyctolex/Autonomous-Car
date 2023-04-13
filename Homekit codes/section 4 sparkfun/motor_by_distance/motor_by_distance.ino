#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 6

#include "src/mapper/mapper.h"
#include "src/motor/motor.h"


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

class DistanceSensorHandler
{
public:
    SFEVL53L1X distanceSensor;
    DistanceSensor()
    {
        Serial.println("VL53L1X Qwiic Test");

        if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
        {
            Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
            while (1)
                ;
        }
        Serial.println("Sensor online!");
    }
    int read()
    {
        distanceSensor.startRanging(); // Write configuration bytes to initiate measurement
        while (!distanceSensor.checkForDataReady())
        {
            delay(1);
        }
        int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();
        return distance;
    }
};

class SensorHandler
{
public:
  static const int sensorMinValue = 0;
  static const int sensorMaxValue = 2500;
  static const int speendMinValue = 0;
  static const int speedMaxValue = 255;
  Map *SpeedScaler;
  DistanceSensorHandler *DistanceSensorHandlerObject ;
  SensorHandler(DistanceSensorHandler* distance_sensor_handler)
  {
    SpeedScaler = new Map(sensorMinValue, sensorMaxValue, speendMinValue, speedMaxValue);
    DistanceSensorHandlerObject  = distance_sensor_handler;
  }

  int get_speed(){
    int sensor_value = DistanceSensorHandlerObject->read();
    int speed = (int)SpeedScaler->map_value((float)sensor_value);
    return speed;
  }
};


#define NUM_LABELS 1
String LABELS[NUM_LABELS] = {"Speed"};
PlotWithLabels *plotter;
DistanceSensorHandler *sensor;
SensorHandler *sensor_handler;
Motor *motor;

void setup(void)
{
    Wire.begin();
    Serial.begin(115200);
    sensor = new DistanceSensorHandler();
    sensor_handler = new SensorHandler(sensor);
    plotter = new PlotWithLabels(LABELS, NUM_LABELS);
    motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
}

void loop(void)
{
    int speed = sensor_handler->get_speed();
    int values[] = {speed};
    plotter->plot(values);
    motor->update(State::forward, speed);
    delay(1);
}

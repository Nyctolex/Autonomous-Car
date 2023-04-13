#include "src/homekitModules.h"

class MagneticEncoder
{
public:
    static volatile bool switchChanged; // declare
    static volatile int encoderCounts;
    static volatile int encoderPreviousCount;
    static volatile unsigned long previous_millis;
    static volatile unsigned long current_millis;
    void attach(int _encoder_pinA, int _encoder_pinB)
    {
        // initialize encoder, attache ISR functions
        pinMode(_encoder_pinA, INPUT);
        pinMode(_encoder_pinB, INPUT);
    }
    void begin()
    {
        encoderCounts = 0;
        // Attached interrupt to encoder pins
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
    }

    static float rpm()
    {
        // return the rpm since last call
        current_millis = millis();
        unsigned long time_delta_millisec = current_millis - previous_millis;
        float rotation_delta = float(encoderCounts - encoderPreviousCount) / 360;
        float time_delta_minues = time_delta_millisec / 60000.0;
        float rpm = (float)rotation_delta / time_delta_minues;
        encoderPreviousCount = encoderCounts;
        previous_millis = millis();
        return rpm;
    }

    // Encoder ISR functions - Interupt Service Routine
    // EncoderA ISR
    static void encoderA()
    {
        // look for a low-to-high on channel B
        if (digitalRead(ENCODER_PINA) == HIGH)
        {
            // check channel A to see which way encoder is turning
            digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;
        }
        else
        {
            // check channel A to see which way encoder is turning
            digitalRead(ENCODER_PINB) ? encoderCounts-- : encoderCounts++;
        }

    } // End EncoderA ISR
    // EncoderB ISR
    static void encoderB()
    {
        // look for a low-to-high on channel B

        if (digitalRead(ENCODER_PINB) == HIGH)
        {
            // check channel A to see which way encoder is turning
            digitalRead(ENCODER_PINA) ? encoderCounts-- : encoderCounts++;
        }
        else
        {
            // check channel A to see which way encoder is turning
            digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--;
        }
    } // End EncoderB ISR
};


// define all class varibles
volatile bool MagneticEncoder::switchChanged;
volatile int MagneticEncoder::encoderCounts;
volatile int MagneticEncoder::encoderPreviousCount;
volatile unsigned long MagneticEncoder::previous_millis;
volatile unsigned long MagneticEncoder::current_millis;
MagneticEncoder magneticSensor; // make an instance of myClass

Motor *motor;
SensorHandler *sensor;
PID_Contrller *controller;
#define NUM_LABELS 3
String LABELS[NUM_LABELS] = {"reference_signal", "RPM", "PWM"};
Plotter *plotter;
double rpm_speed;
int speed;
double reference_signal = 0;
void setup()
{
    motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
    motor->update(State::coast, 0);
    sensor = new SensorHandler(A0, 0.3);
    controller = new PID_Contrller(2,1000,0,reference_signal,-255,255);
    plotter = new Plotter(LABELS, NUM_LABELS);
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    magneticSensor.attach(ENCODER_PINA, ENCODER_PINB);
    magneticSensor.begin();
    speed = 0;
    motor->update(State::forward, speed);
    for (int i=0;i<20;i++){
          int values[] = {0, 0};

    plotter->plot(values);
    delay(50);
    }
}

// the loop routine runs over and over again forever:
void loop()
{
    // int speed = sensor->get_speed();
    // int state = sensor->get_state();
    reference_signal = (double)sensor->get_speed();
    controller->target_value = reference_signal;
    double rpm_speed = (double)magneticSensor.rpm();
    double pid_value = controller->next(rpm_speed);
    int error_to_pid = (int) ((pid_value - speed)*0.3);
    speed += error_to_pid;
    motor->update(State::forward, speed);
    int values[] = {(int)reference_signal,  (int)rpm_speed, (int)speed };
    plotter->plot(values);
    delay(100);// delay in between reads for stability
}
// Encoder Example

// Define Pins

#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define RED_LED_PIN 9
#define BLUE_LED_PIN 11
#define GREEN_LED_PIN 10
#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 4

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

// the diffrent state the motor could be in
enum State
{
    forward,
    backward,
    coast,
    halt
};

class Map
{
    // A class that linearly maps between input range of values to output range of values
public:
    float input_min_value;
    float input_max_value;
    float output_min_value;
    float output_max_value;
    float input_scale_size;
    float output_scale_size;
    Map(float input_min_value, float input_max_value, float output_min_value, float output_max_value)
    {
        this->input_min_value = input_min_value;
        this->input_max_value = input_max_value;
        this->output_min_value = output_min_value;
        this->output_max_value = output_max_value;
        this->input_scale_size = input_max_value - input_min_value;
        this->output_scale_size = output_max_value - output_min_value;
    }
    float map_value(float input_value)
    {
        // remap the analog value to a value between 0 to 1
        float zero_to_one_scale = (input_value - this->input_min_value) / this->input_scale_size;
        float voltage_value = zero_to_one_scale * this->output_scale_size + this->input_min_value;
        return voltage_value;
    }
};

class Motor
{
    // Handle the motor speed and state
public:
    int clockwise_pin, counter_clockwise_pin;
    Motor(int clockwise_pin, int counter_clockwise_pin)
    {
        this->clockwise_pin = clockwise_pin;
        this->counter_clockwise_pin = counter_clockwise_pin;
        pinMode(this->clockwise_pin, OUTPUT);
        pinMode(this->counter_clockwise_pin, OUTPUT);
    }

    void update(int state, int speed)
    {
        switch (state)
        {
        case State::forward:
            digitalWrite(this->counter_clockwise_pin, LOW);
            analogWrite(this->clockwise_pin, speed);
            break;
        case State::backward:

            digitalWrite(this->clockwise_pin, LOW);
            analogWrite(this->counter_clockwise_pin, speed);
            break;
        case State::halt:
            digitalWrite(this->counter_clockwise_pin, HIGH);
            digitalWrite(this->clockwise_pin, HIGH);
            break;
        default:
            // the default is coast
            digitalWrite(this->counter_clockwise_pin, LOW);
            digitalWrite(this->clockwise_pin, LOW);
        }
    }
};

class SensorHandler
{
public:
    static const int sensorMinValue = 0;
    static const int sensorMaxValue = 1023;
    static const int speendMinValue = 0;
    static const int speedMaxValue = 255;
    int scaleSize = sensorMaxValue - sensorMinValue;
    int middleStartThreshole, middleEndThreshole;
    Map *SpeedScaler;
    byte SensorPin;
    SensorHandler(byte sensor_pin, float middleThreshole)
    {
        float middleValue = (sensorMinValue + sensorMaxValue) / 2;
        middleStartThreshole = (int)((float)middleValue - (float)scaleSize * middleThreshole / 2);
        middleEndThreshole = (int)((float)middleValue + (float)scaleSize * middleThreshole / 2);
        int Sensor_pin = sensor_pin;
        SpeedScaler = new Map(sensorMinValue, middleStartThreshole, speendMinValue, speedMaxValue);
    }
    int get_speed()
    {
        int sensorValue = analogRead(A0);
        int unscaled_speed;
        if (sensorValue < middleStartThreshole)
        {
            unscaled_speed = middleStartThreshole - sensorValue;
        }
        else if (sensorValue > middleEndThreshole)
        {
            unscaled_speed = sensorValue - middleEndThreshole;
        }
        else
        {
            unscaled_speed = 0;
        }
        int scale_speed = (int)SpeedScaler->map_value((float)unscaled_speed);
        return scale_speed;
    }
    int get_state()
    {
        int sensorValue = analogRead(A0);

        if (Serial.availableForWrite() > 0)
        {
        }
        if (sensorValue < middleStartThreshole)
        {
            return State::forward;
        }
        else if (sensorValue > middleEndThreshole)
        {
            return State::backward;
        }
        else
        {
            return State::coast;
        }
    }
};

class LedHandler
{
public:
    int forward_led, backward_led, halt_led;
    LedHandler(int forward_led, int backward_led, int stop_led)
    {
        forward_led = forward_led;
        backward_led = backward_led;
        halt_led = stop_led;
        pinMode(forward_led, OUTPUT);
        pinMode(backward_led, OUTPUT);
        pinMode(halt_led, OUTPUT);
    }

    void update(int state)
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
};

// define all class varibles
volatile bool MagneticEncoder::switchChanged;
volatile int MagneticEncoder::encoderCounts;
volatile int MagneticEncoder::encoderPreviousCount;
volatile unsigned long MagneticEncoder::previous_millis;
volatile unsigned long MagneticEncoder::current_millis;
MagneticEncoder magneticSensor; // make an instance of myClass

Motor *motor;
LedHandler *leds;
SensorHandler *sensor;
void setup()
{
    motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
    motor->update(State::coast, 0);
    leds = new LedHandler(BLUE_LED_PIN, RED_LED_PIN,GREEN_LED_PIN );
    sensor = new SensorHandler(A0, 0.3);

    // initialize serial communication at 115200 bits per second:

    Serial.begin(115200);
    magneticSensor.attach(ENCODER_PINA, ENCODER_PINB);
    magneticSensor.begin();
}

// the loop routine runs over and over again forever:
void loop()
{
    int speed = sensor->get_speed();
    int state = sensor->get_state();
    motor->update(state, speed);
    leds->update(state);
    Serial.print("RPM:");
    Serial.println(magneticSensor.rpm());
    delay(100);// delay in between reads for stability
}

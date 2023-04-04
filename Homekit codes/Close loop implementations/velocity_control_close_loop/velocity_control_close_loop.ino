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

class PID_Contrller{
  public:
  double Kp;
  double Ki;
  double Kd;
  double target_value;
  double target_min_value, target_max_value, target_value_range;
  float cumulative_error; // units are cm, this is needed for I-term in PID 
  double inetgrated_error;
double previous_error; // this is for D term in PID
double dt;
double prev_time;
Map *mapper;
PID_Contrller(double kp, double ki, double kd, double target_value, double target_min_value, double target_max_value){
  this->Kp = kp;
  this->Ki = ki;
  this->Kd = kd;
  this->target_value = target_value;
  this->target_min_value = target_min_value;
  this->target_max_value = target_max_value;
  this->target_value_range = target_max_value - target_min_value;
  this->reset_controller();
  this->mapper = new Map(0, target_max_value, target_min_value, target_max_value);
}
void reset_controller(){
  this->cumulative_error = 0;
  this->inetgrated_error = 0;
  this->previous_error = 0;
  this->prev_time = millis();
}

double next(double sensor_output){
    // input == distance from proximity sensor to ball
  // output == new angle to move the servo motor to get ball closer to setpoint
  // never let the ball get closer than 4 cm to the proximity sensor - else it isn't accurate
  this->dt = (double)(millis()-this->prev_time)/ 1000000.0;
  double error_value = this->target_value - sensor_output;

  double p_value = error_value * this->Kp;

  double i_value = this->inetgrated_error *this->Ki;

  double d_value = (error_value - this->previous_error) * this->Kd / this->dt;
  // typically we would divide by the elapsed time as the D-term is checking the error rate
  // so if the ball is moving really fast the wrong way (away from setpoint), the correction will be bigger than P-only
  
  double pid_value = p_value + d_value+ i_value;

if (!isnan(this->dt*error_value)) {
this->inetgrated_error += this->dt*error_value;
}

  
  // this->cumulative_error += error; // note that error can be + or -, this i term seeks to eliminate the offset when the P-only controller stalls
  this->previous_error = error_value; // for the next cycle, remember what this cycle's error was
  // map the pid value to a new angle for the servo to go to
  double new_value = pid_value;//this->mapper->map_value(pid_value);
  if (new_value < this->target_min_value){
    return this->target_min_value;
  }
  if (new_value > this->target_max_value){
    return this->target_max_value;
  }
  this->prev_time = millis();
  return new_value;
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
SensorHandler *sensor;
PID_Contrller *controller;
#define NUM_LABELS 4
String LABELS[NUM_LABELS] = {"Wanted_value", "RPM", "error"};
PlotWithLabels *plotter;
double rpm_speed;
int speed;
double wanted_value = 300;
void setup()
{
    motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
    motor->update(State::coast, 0);
    sensor = new SensorHandler(A0, 0.3);
    controller = new PID_Contrller(2,1000,0.0,wanted_value,0,255);
    plotter = new PlotWithLabels(LABELS, NUM_LABELS);
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    magneticSensor.attach(ENCODER_PINA, ENCODER_PINB);
    magneticSensor.begin();
    speed = 0;
    motor->update(State::forward, speed);
    for (int i=0;i<300;i++){
          int values[] = {0, 0, (int)wanted_value};

    plotter->plot(values);
    }
}

// the loop routine runs over and over again forever:
void loop()
{
    // int speed = sensor->get_speed();
    // int state = sensor->get_state();
    double rpm_speed = (double)magneticSensor.rpm();
    double pid_value = controller->next(rpm_speed);
    int error_to_pid = (int) ((pid_value - speed)*0.3);
    speed += error_to_pid;
    motor->update(State::forward, speed);
    int values[] = {(int)wanted_value,  (int)rpm_speed, (int)(wanted_value - rpm_speed)};

    plotter->plot(values);
    delay(100);// delay in between reads for stability
}
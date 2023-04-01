// Encoder Example

// Define Pins

#define ENCODER_PINA 2
#define ENCODER_PINB 3

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

void setup()
{

    // initialize serial communication at 115200 bits per second:

    Serial.begin(115200);
    magneticSensor.attach(ENCODER_PINA, ENCODER_PINB);
    magneticSensor.begin();
    Serial.print("Encoder_Value");
}

void loop()
{
    Serial.print("RPM:")
    Serial.println(magneticSensor.rpm());
    delay(10);
}

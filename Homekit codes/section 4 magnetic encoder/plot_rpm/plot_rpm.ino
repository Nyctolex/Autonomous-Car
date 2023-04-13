// Encoder Example

// Define Pins

#define ENCODER_PINA 2
#define ENCODER_PINB 3

// encoder variables
volatile int encoderCounts = 0;

// Encoder ISR functions - Interupt Service Routine

void encoderA();

void encoderB();

unsigned long previous_millis;
unsigned long current_millis;
int encoderPreviousCount;
void setup()
{

    // initialize serial communication at 115200 bits per second:

    Serial.begin(115200);

    // initialize encoder, attache ISR functions

    pinMode(ENCODER_PINA, INPUT);

    pinMode(ENCODER_PINB, INPUT);

    // Attached interrupt to encoder pins

    attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);

    Serial.print("Encoder_Value");
}

void loop()
{

    // print encoder position
    current_millis = millis();
    unsigned long time_delta_millisec = current_millis - previous_millis;
    float rotation_delta = float(encoderCounts - encoderPreviousCount) / 360;
    float time_delta_minues = time_delta_millisec / 60000.0;
    float rpm = (float)rotation_delta / time_delta_minues;

    Serial.print("rpm:");
    Serial.println(rpm);
    encoderPreviousCount = encoderCounts;
    previous_millis = millis();
    delay(100);
}

// EncoderA ISR
void encoderA()
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
void encoderB()
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
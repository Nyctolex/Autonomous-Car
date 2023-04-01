#define CLOCKWISE_PIN 5
#define COUNTERCLOCKWISE_PIN 4

// the diffrent state the motor could be in
enum State
{
  forward,
  backward,
  coast,
  halt
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

Motor *motor;
int speed;
int state;
void setup()
{
  Serial.begin(9600);
  motor = new Motor(CLOCKWISE_PIN, COUNTERCLOCKWISE_PIN);
  motor->update(State::coast, 0);
}

// the loop routine runs over and over again forever:
void loop()
{
    speed = 255;
  state = State::forward;
  motor->update(state, speed);
  delay(5000);

      speed = 255;
  state = State::halt;
  motor->update(state, speed);
  delay(5000);

      speed = 255;
  state = State::backward;
  motor->update(state, speed);
  delay(5000);

      speed = 255;
  state = State::halt;
  motor->update(state, speed);
  delay(5000);
}
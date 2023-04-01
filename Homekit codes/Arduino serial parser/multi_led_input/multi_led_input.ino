#define RED_LED_PIN 9
#define BLUE_LED_PIN 11
#define GREEN_LED_PIN 10

void RGB_string_to_int(String message, int *target_array)
{
  char buffer[50];
  message.toCharArray(buffer, 50);
  char *token;
  char *rest = buffer;
  int i = 0;

  while ((token = strtok_r(rest, ",", &rest)))
  {
    target_array[i] = atoi(token);
    i++;
  }
}

int RGB_brightness_values[] = {0, 0, 0};
void setup()
{
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
}

void loop()
{

  if (Serial.available() > 0)
  {
    RGB_string_to_int(Serial.readString(), RGB_brightness_values);
    analogWrite(RED_LED_PIN, RGB_brightness_values[0]);
    analogWrite(GREEN_LED_PIN, RGB_brightness_values[1]);
    analogWrite(BLUE_LED_PIN, RGB_brightness_values[2]);
  }
  delay(10);
}

/*
  Odometry && Gyro Integration with teleoperate
*/
#include <Wire.h>
#include <Zumo32U4.h>

// zumo classes
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

// time variables
#define SAMPLERATE 10          // 5 millis =  200 Hz
// Odometry settings
#define GEAR_RATIO 75      // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98    // Distance between tracks
#define WHEEL_DIAMETER 37.5   // Wheels diameter measured 38.5
#define ENCODER_PPR 12        // Encoder pulses per revolution
#define GYRO_SCALE (90.0/1283.0 / 717.0)*90        // 70 mdps/LSB 
float encoder2dist = WHEEL_DIAMETER*3.14/(ENCODER_PPR*GEAR_RATIO);  // conversition of encoder pulses to distance in mm

//
class GyroHandler
{
    private:
    // imu Fusion
    float gyroAngle = 0;
    int32_t gyroOffset_z = -16;
    float gyroz = 0;
    unsigned long lastMillis = 0;
    unsigned long lastMicros = 0;

public:
 float dt_time;
 
    GyroHandler()
    {
      if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  imu.enableDefault();
//        imu.configureForTurnSensing();

        this->gyroAngle = 0;
//        this->gyroOffset_z = -16;
        this->gyroz = 0;
        // take time stamp
        this->lastMillis = millis();
        this->lastMicros = micros();
        this->gyroOffset();
        this->angleOffset();
    }

    // gyro calibration
    void gyroOffset()
    {
        delay(1); // delay before starting gyro readings for offset
        int32_t total = 0;
        for (uint16_t i = 0; i < 1024; i++)
        {
            // Wait for new data to be available, then read it.
            while (!imu.gyroDataReady())
            {
            }
            imu.readGyro();

            // Add the Z axis reading to the total.
            total += imu.g.z;
        }
        this->gyroOffset_z = total / 1024;
    }


        // gyro calibration
    void angleOffset()
    {
        delay(1); // delay before starting gyro readings for offset
        float total = 0;
        for (uint16_t i = 0; i < 1024; i++)
        {
            total += this->gyroIntegration(true);
        }
        float angleOffset_z = total / 1024;
        this->gyroAngle -= angleOffset_z;
    }

    // gyroIntegration
    float gyroIntegration(bool motorsState)
    {
        this->update_dt();
        imu.readGyro();
        this->gyroz = ((float)(imu.g.z - (float)this->gyroOffset_z))* GYRO_SCALE;
        if (motorsState)
            this->gyroAngle += (gyroz * this->dt_time); // integrate when in motion
        return this->gyroAngle;
    }

    void update_dt()
    {
        this->lastMillis = millis();
        // calculate dt sample
        unsigned long dtMicros = micros() - lastMicros;
        this->lastMicros = micros();
        this->dt_time = float(dtMicros) / 1000000.0;
    }


};


class OdometryHandler
{
public:
    float theta;
    float posx;
    float posy;
    OdometryHandler()
    {
        this->theta = 0;
        this->posx = 0;
        this->posy = 0;
    }

    void odometry(boolean motorsState)
    {
        // encoder read
        int16_t countsLeft = encoders.getCountsAndResetLeft();
        int16_t countsRight = encoders.getCountsAndResetRight();
        float dx_1 = countsRight * encoder2dist;
        float dx_2 = countsLeft * encoder2dist;
        float d_theta = float(dx_1 - dx_2) / WHEELS_DISTANCE;
        this->posx += cos(theta + d_theta / 2) * (dx_1 + dx_2) / 2;
        this->posy += sin(theta + d_theta / 2) * (dx_1 + dx_2) / 2;
        this->theta += d_theta;
    }
};


OdometryHandler * odometryHandler;
GyroHandler * gyroHandler;
boolean motorsState = 0;

void setup(){
  Wire.begin();
    gyroHandler = new GyroHandler();
   
    odometryHandler = new OdometryHandler();
    // initialize serial:
  Serial.begin(9600);

}

void loop(){

     // update motors 
     int leftMotor = 100;
     int rightMotor = 100;
    motors.setLeftSpeed(leftMotor);
    motors.setRightSpeed(rightMotor);
    motorsState = (leftMotor || rightMotor) ==  0 ? 0 : 1; //  check if motors are still
    float gyroAngle = gyroHandler->gyroIntegration(motorsState);
    odometryHandler->odometry(motorsState);



    // send a response
//    Serial.print(leftMotor);
//    Serial.print(" , ");
//    Serial.print(rightMotor);
//    Serial.print(" , ");
//    Serial.print(gyroHandler->dt_time);
//    Serial.print(" , ");
//    Serial.print(odometryHandler->posx);
//    Serial.print(" , ");
//    Serial.print(odometryHandler->posy);
//    Serial.print(" , ");
//    Serial.print(odometryHandler->theta*57.295);
//    Serial.print(" , ");
    Serial.print(gyroAngle);
Serial.println("");

}

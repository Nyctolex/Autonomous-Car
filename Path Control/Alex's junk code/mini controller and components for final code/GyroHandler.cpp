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
#define SAMPLERATE 10 // 5 millis =  200 Hz
#define GYRO_SCALE 0.07        // 70 mdps/LSB 

class GyroHandler
{
public:
    GyroHandler()
    {
        imu.init();
        imu.enableDefault();
        imu.configureForTurnSensing();

        this->gyroAngle = 0;
        this->gyroOffset_z = -16;
        this->gyroz = 0;
        // take time stamp
        this->lastMillis = millis();
        this->lastMicros = micros();
        this->gyroOffset();
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

    // gyroIntegration
    void gyroIntegration(bool motorsState)
    {
        this->update_dt();
        imu.readGyro();
        this->gyroz = ((float)(imu.g.z - (int16_t)this->gyroOffset_z)) * GYRO_SCALE;
        if (motorsState)
            gyroAngle += (gyroz * this->dt_time); // integrate when in motion
    }

    void update_dt()
    {
        this->lastMillis = millis();
        // calculate dt sample
        unsigned long dtMicros = micros() - lastMicros;
        this->lastMicros = micros();
        dt_time = float(dtMicros) / 1000000.0;
    }

private:
    // imu Fusion
    float gyroAngle = 0;
    int32_t gyroOffset_z = -16;
    float gyroz = 0;
    unsigned long lastMillis = 0;
    unsigned long lastMicros = 0;
    // boolean motorsState = 0;
}
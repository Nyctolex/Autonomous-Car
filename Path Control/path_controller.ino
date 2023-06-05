#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846 /* pi */

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define PATH_LENGTH 6

float path[PATH_LENGTH][2] = {
    {0, 0},
    {1, 2},
    {1, 5},
    {5, 5},
    {5, 10},
    {1, 1}};

/*
  Odometry && Gyro Integration with teleoperate
*/
#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

// zumo classes
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

// time variables
#define SAMPLERATE 10 // 5 millis =  200 Hz
// Odometry settings
#define GEAR_RATIO 75                                                    // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98                                               // Distance between tracks
#define WHEEL_DIAMETER 37.5                                              // Wheels diameter measured 38.5
#define ENCODER_PPR 12                                                   // Encoder pulses per revolution
#define GYRO_SCALE 360 / 50829.33                                        // 70 mdps/LSB
float encoder2dist = WHEEL_DIAMETER * 3.14 / (ENCODER_PPR * GEAR_RATIO); // conversition of encoder pulses to distance in mm

class PIDController
{
private:
    float kp;                  // Proportional gain
    float ki;                  // Integral gain
    float kd;                  // Derivative gain
    float Ts;                  // Sampling time
    float integral;            // Integral accumulator
    float prev_error;          // Error from previous iteration
    float prev_derivative;     // Derivative from previous iteration
    float derivative;          // Current derivative value
    float alpha;               // Filter coefficient
    float filtered_derivative; // Filtered derivative value
    unsigned long last_time;

public:
    // Constructor
    PIDController(float Kp, float Ki, float Kd, float Alpha)
    {
        this->kp = Kp;
        this->ki = Ki;
        this->kd = Kd;
        this->Ts = 0;
        this->alpha = Alpha;
        this->integral = 0.0;
        this->prev_error = 0.0;
        this->prev_derivative = 0.0;
        this->derivative = 0.0;
        this->filtered_derivative = 0.0;
        this->last_time = 0;
    }

    // PID control function
    float pid_control(float setpoint, float process_variable)
    {

        // Calculate time since last sample
        unsigned long current_time = millis();
        float dt = (current_time - last_time) / 1000.0; // convert to seconds
        this->last_time = current_time;

        // Update Ts
        this->Ts = dt;

        // Calculate error
        float error = setpoint - process_variable;

        // Calculate integral term
        this->integral += error * this->Ts;

        // Calculate derivative term with FIR filter
        this->derivative = (error - this->prev_error) / this->Ts;
        this->filtered_derivative = this->alpha * this->derivative + (1 - this->alpha) * this->prev_derivative;

        // Calculate control signal
        float control_signal = this->kp * error + this->ki * this->integral + this->kd * this->filtered_derivative;

        // Store previous error and derivative for next iteration
        this->prev_error = error;
        this->prev_derivative = this->filtered_derivative;

        return control_signal;
    }
};

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

            Serial.println(F("Failed to initialize IMU sensors."));
            delay(100);
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
        //(90.0/1283.0 / 717.0)*90
        this->update_dt();
        imu.readGyro();
        this->gyroz = ((float)(imu.g.z - (float)this->gyroOffset_z)) * GYRO_SCALE;
        if (motorsState)
            this->gyroAngle += (float)(this->gyroz * this->dt_time); // integrate when in motion
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
    bool external_theta;
    OdometryHandler(bool external_theta = true)
    {
        this->theta = 0;
        this->posx = 0;
        this->posy = 0;
        this->external_theta = external_theta;
    }

    void odometry(bool motorsState, float new_theta = 0)
    {
        // encoder read
        int16_t countsLeft = encoders.getCountsAndResetLeft();
        int16_t countsRight = encoders.getCountsAndResetRight();
        float dx_1 = countsRight * encoder2dist;
        float dx_2 = countsLeft * encoder2dist;
        float d_theta;
        if (this->external_theta)
        {
            d_theta = new_theta - this->theta;
        }
        else
        {
            d_theta = float(dx_1 - dx_2) / WHEELS_DISTANCE;
        }

        this->posx += cos(theta + d_theta / 2) * (dx_1 + dx_2) / 2;
        this->posy += sin(theta + d_theta / 2) * (dx_1 + dx_2) / 2;
        this->theta += d_theta;
    }
};

class PositionHandler
{
private:
    float theta;
    float posx;
    float posy;
    bool external_theta;
    OdometryHandler *odometryHandler;
    GyroHandler *gyroHandler;

public:
    PositionHandler(bool external_theta = true)
    {
        this->theta = 0;
        this->posx = 0;
        this->posy = 0;
        this->external_theta = external_theta;
        this->gyroHandler = new GyroHandler();
        this->odometryHandler = new OdometryHandler();
    }

    void update(bool motorsState)
    {
        this->theta = gyroHandler->gyroIntegration(motorsState);
        if (this->external_theta)
        {
            odometryHandler->odometry(motorsState, this->theta);
        }
        else
        {
            odometryHandler->odometry(motorsState);
        }
        this->posx = this->odometryHandler->posx;
        this->posy = this->odometryHandler->posy;
    }

    float getx(bool motorsState)
    {
        this->update(motorsState);
        return this->posx;
    }

    float gety(bool motorsState)
    {
        this->update(motorsState);
        return this->posy;
    }
    float getTheta(bool motorsState)
    {
        this->update(motorsState);
        return this->theta;
    }
};

enum rotateAxis
{
    middle = 0,
    right = 1,
    left = 2
};

class WheelsSpeed
{
public:
    float right, left;

public:
    WheelsSpeed(float rigt, float left)
    {
        this->right = right;
        this->left = left;
    }
};

class Vector2D
{
public:
    float x;
    float y;

    // Constructor
    Vector2D(float x_val = 0.0f, float y_val = 0.0f) : x(x_val), y(y_val) {}

    // Subtraction
    Vector2D operator-(const Vector2D &other) const
    {
        return Vector2D(x - other.x, y - other.y);
    }

    // Addition
    Vector2D operator+(const Vector2D &other) const
    {
        return Vector2D(x + other.x, y + other.y);
    }
    // Scalar multiplication
    Vector2D operator*(float scalar) const
    {
        return Vector2D(x * scalar, y * scalar);
    }
    // Scalar division
    Vector2D operator/(float scalar) const
    {
        return Vector2D(x / scalar, y / scalar);
    }

    float operator&(const Vector2D &other) const
    {

        return (x * other.x + y * other.y);
    }

    // Norm calculation
    float norm() const
    {
        return sqrt(x * x + y * y);
    }

    bool is_clock_wise_angle(const Vector2D &other)
    {
        // returns if v1 is clockwise to v2
        Vector2D v1 = this->hat();
        Vector2D v2 = other.hat();
        float det = v1.x * v2.y - v1.y * v2.x;
        return det < 0;
    }

    // normilized version of the vector
    Vector2D hat() const
    {
        return Vector2D(x, y) / this->norm();
    }

    bool is_zero() const
    {
        return ((x == 0) && (y == 0));
    }
};

class Line
{
public:
    Vector2D p1;
    Vector2D p2;
    Vector2D pointing;
    // Constructor
    Line(float x1, float y1, float x2, float y2)
    {
        this->p1 = Vector2D(x1, y1);
        this->p2 = Vector2D(x2, y2);
        this->pointing = p2 - p1;
    }
    Line(Vector2D p1, Vector2D p2)
    {
        this->p1 = p1;
        this->p2 = p2;
        this->pointing = p2 - p1;
    }
};

float line_point_distance(Line line, Vector2D point)
{
    Vector2D p = line.pointing;
    Vector2D n(-p.y, p.x);
    return p & (n.hat());
}

float distance_to_end_section(Line line, Vector2D point)
{
    Vector2D p = line.pointing;
    Vector2D p3 = line.p2 - point;
    return p.hat() & p3;
}

bool passed_section(Line line, Vector2D point)
{
    return distance_to_end_section(line, point) < 0;
}

bool about_to_pass_section(Line section, Vector2D point, float threshole = 0.05)
{
    return distance_to_end_section(section, point) < threshole;
}

int find_next_section(float path[][2], Vector2D position, int current_index, float threshold)
{
    Vector2D p1(path[current_index][0], path[current_index][1]);
    Vector2D p2(path[current_index + 1][0], path[current_index + 1][1]);
    Line line(p1, p2);

    if (current_index == PATH_LENGTH - 2)
        return current_index;

    bool have_passed_section = passed_section(line, position);

    if (have_passed_section)
        return (current_index + 1);
    if ((!have_passed_section) && (distance_to_end_section(line, position) < threshold))
        return (current_index + 1);
    return current_index;
}

bool is_clock_wise_angle(Vector2D v1, Vector2D v2)
{
    // returns if v1 is clockwise to v2
    v1 = v1.hat();
    v2 = v2.hat();

    float det = v1.x * v2.y - v1.y * v2.x;
    return det < 0;
}

float non_negative_angle(float angle)
{
    // Transfer a negative angle to it's positive version (in radians)
    angle = fmod(angle, (2 * PI));
    return (angle > 0) ? angle : 2 * PI + angle;
}

float inner_angle(Vector2D v1, Vector2D v2 = Vector2D(0, 0))
{
    // Returns the inner angle between v1 and v2. By default, it would be the angle with the x axis

    float angle1 = non_negative_angle(atan2(v1.y, v1.x));
    if (v2.is_zero())
        return angle1;
    float angle2 = non_negative_angle(atan2(v2.y, v2.x));
    float angle = max(angle1, angle2) - min(angle1, angle2);
    return (angle < PI) ? angle : (2 * PI - angle);
}

float next_point_controller(Vector2D point, Vector2D position, Vector2D velocity_vector)
{
    // Path controller which return the next direction for the car
    Vector2D pointing_vector = point - position;
    float angle = inner_angle(velocity_vector, pointing_vector);
    return (is_clock_wise_angle(pointing_vector, velocity_vector)) ? angle : -1 * angle;
}

Line getSection(float path[][2], int index)
{
    Vector2D p1(path[index][0], path[index][1]);
    Vector2D p2(path[index + 1][0], path[index + 1][1]);
    return Line(p1, p2);
}

bool finished(Line section, Vector2D position, int index, float threshole = 0.05)
{
    if (index + 2 < PATH_LENGTH)
        return false;
    return about_to_pass_section(section, position, threshole);
}

class Car
{
private:
    Vector2D position;
    float direction;
    float velocity;
    float target_direction;
    float target_velocity;

public:
    PositionHandler *positionHandler;
    int leftMotor, rightMotor;
    bool motorsState;
    PIDController distance_pid_controller;
    PIDController angle_pid_controller;
    PIDController velocity_pid_controller;
    // Constructor
    Car(const Vector2D &initial_position, float initial_direction, float initial_velocity)
        : position(initial_position), direction(initial_direction), velocity(initial_velocity) {}
    Car(const Vector2D initial_position, const Vector2D initial_velocity)
    {
        this->distance_pid_controller = PIDController(0.1, 0.01, 0, 0.1);
        this->angle_pid_controller = PIDController(3, 0.2, 0, 0.1);
        this->velocity_pid_controller = PIDController(400, 0.1, 0, 0.1);
        this->position = initial_position;
        this->direction = inner_angle(initial_velocity);
        this->velocity = initial_velocity.norm();
        this->positionHandler = new PositionHandler(true);
        this->leftMotor = 0;
        this->rightMotor = 0;
    }
    Vector2D get_position(){
        return this->position;
    }
    float get_velocity()
    {
        return this->velocity;
    }

        float get_direction()
    {
        return this->direction;
    }
        Vector2D get_velocity_vector() const
    {
        return Vector2D(cos(this->direction), sin(this->direction)) * this->velocity;
    }

    void set_velocity(float velocity)
    {
        this->target_velocity = velocity;
    }
    void set_direction(float theta)
    {
        this->target_direction = theta;
    }



    void update_position(float dt)
    {
        motorsState = (leftMotor || rightMotor) == 0 ? 0 : 1;
        this->direction = positionHandler->getTheta(motorsState);
        float x = = positionHandler->getx(motorsState);
        float y = = positionHandler->gety(motorsState);
        this->position = Vector2D(x, y);

        int16_t leftCount = encoders.getCountsAndResetLeft();
        int16_t RightCount = encoders.getCountsAndResetRight();
        this->velocity = ((leftCount + RightCount) / 2) * encoder2dist / Ts;
        float velocity_control_signal = velocity_pid_controller.pid_control(this->velocity, this->target_velocity);
        float angle_control_signal = angle_pid_controller.pid_control(this->target_direction, this->direction);

        switch (axis)
        {
        case right:
            this->leftMotor = 0;
            this->rightMotor = (int)angle_control_signal;
            break;
        case left:
            // Apply the control signal to the motors
            this->leftMotor = (int)-1 * angle_control_signal;
            this->rightMotor = 0;

            break;

        default:
            // Apply the control signal to the motors
            this->leftMotor = (int)-1 * angle_control_signal / 2;
            this->rightMotor = (int)angle_control_signal / 2;
            break;
        }
        this->leftMotor = (int)((velocity_control_signal + leftMotor) / 2);
        this->rightMotor = (int)((velocity_control_signal + rightMotor) / 2);
        motors.setLeftSpeed(this->leftMotor);
        motors.setRightSpeed(this->rightMotor);
        delay(dt * 1000);
    }
};

enum CarState
{
    driving,
    rotating,
    smooth,
    done
};
// if it should move as a polygon motion or smooth
bool polygon = false;
// other intial states
/// Start ///
bool motorsState;
float Ts;
int axis = middle;

int carState = CarState::rotating;
bool started = false;
Vector2D initial_pos(0, 0);
int section_index = 0;

Vector2D initial_velocity(-1, 0);
float velocity = initial_velocity.norm();
Vector2D current_pos(0, 0);
float dt = 0.1;
Car car(initial_pos, initial_velocity);
Line current_section = getSection(path, section_index);
float new_direction;

void print_vec(Vector2D vec)
{
    Serial.print(vec.x);
    Serial.print(',');
    Serial.println(vec.y);
}

void setup()
{
    // initialize serial:
    Serial.begin(9600);
    Wire.begin();
    // initialize serial:
    Ts = 0.01;
}

bool init_connection()
{
    if ((Serial.available() > 0) && !started)
    {
        String teststr = Serial.readString();
        if (teststr.indexOf("run") >= 0)
        {
            started = true;
            Serial.println("running");
        }
    }
}

void smooth_motion(float pass_section_threshole = 0.05)
{
    // run simulation
    if (finished(current_section, car.position, section_index))
        carState = CarState::done;

    // find next section
    if (about_to_pass_section(current_section, car.position, pass_section_threshole) && ((section_index + 2) < PATH_LENGTH))
    {
        section_index++;
        carState = CarState::rotating;
        current_section = getSection(path, section_index);
    }
    new_direction = next_point_controller(current_section.p2, car.position, car.get_velocity_vector());
    car.set_direction(car.get_direction() + new_direction / 2);
}

void polygon_motion()
{
    float epsilon = 1;
    // run simulation
    // set velocity to zero and roatate car
    if (carState == CarState::driving)
    {
        car.velocity = velocity;
        smooth_motion(0);
    }
    if (carState == CarState::rotating)
    {
        Serial.println("debug: rotating");
        new_direction = next_point_controller(current_section.p2, car.position, car.get_velocity_vector());
        float target_angle = car.get_direction() + new_direction;
        car.velocity = 0;
        car.set_direction(target_angle);
        if (fmod(abs(car.direction - target_angle), (float)(2 * PI)) < epsilon)
            carState = driving;
    }

    print_vec(car.position);
    car.update_position(dt);
}

void loop()
{
    if (carState != CarState::done)
    {

        if (started)
        {
            if (polygon)
            {
                polygon_motion();
            }
            else
            {
                smooth_motion();
            }
            car.update_position(dt);
            print_vec(car.position);
        }
        else
        {
            init_connection();
        }
    }
    else
    {
        Serial.println("done");
    }

    delay(10);
}
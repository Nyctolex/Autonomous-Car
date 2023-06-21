    #include <stdio.h>
#include <math.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>
// zumo classes
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
#define PI 3.14159265358979323846 /* pi */

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define PATH_MAX_LENGTH 50


// uncomment for squre 
// if it should move as a polygon motion or smooth

enum PathShape
{
    squareShape = 0,
    circleShape = 1
};

int path_length = 10;
int path_shape = PathShape::squareShape;
bool polygon = false;
// #define PATH_LENGTH 5
// float path[PATH_LENGTH][2] = {
//     {0, 0},
//     {0, 0.3},
//     {-0.3, 0.3},
//     {-0.3, 0},
//     {0, 0}
//     };

float path[PATH_MAX_LENGTH][2];

//uncomment for circle
// if it should move as a polygon motion or smooth
// float ** create_circle_path(int radius, int resolution){

//     float angle_spacing = 2*PI / resolution;
//     float angle = 0;
//     for (int i = 0; i < resolution;i++){
//         angle += angle_spacing;
//     }
// }

// bool polygon = true;
// int path_length = 6;
// float path[MAX_PATH_LENGTH][2] ={{0.0,0.0},
// {-0.207,0.285},
// {-0.543,0.176},
// {-0.543,-0.176},
// {-0.207,-0.285},
// {0.0,-0.0},
// };



/*
  Odometry && Gyro Integration with teleoperate
*/



// time variables
#define SAMPLERATE 10 // 5 millis =  200 Hz
// Odometry settings
#define GEAR_RATIO 75                                                    // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98                                               // Distance between tracks
#define WHEEL_DIAMETER 37.5                                              // Wheels diameter measured 38.5
#define ENCODER_PPR 12                                                   // Encoder pulses per revolution
#define GYRO_SCALE 2*PI / 2089.56                                     // 70 mdps/LSB
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
        this->integral = 0;
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
        this->integral += error * dt;

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
    int32_t gyroOffset_z = 0;
    float gyroz = 0;
    unsigned long lastMillis = 0;
    unsigned long lastMicros = 0;

public:
    float dt_time;

    GyroHandler()
    {
        // imu.init();
        // imu.enableDefault();
        // imu.configureForTurnSensing();
        this->gyroAngle = 0;
        this->gyroz = 0;
        // take time stamp
        this->lastMillis = millis();
        this->lastMicros = micros();
        this->gyroOffset();
        this->gyroAngle = PI/2;
    }

    reset(float angle){
        this->gyroAngle = angle;
        this->lastMicros = micros();
    }

    // gyro calibration
    void gyroOffset()
    {
        int32_t total = 0;
        for (int i = 0; i < 2048*2; i++)
        {
            // // Wait for new data to be available, then read it.
            if (!imu.gyroDataReady())
            {
                Serial.println("There is an error with the gyro");
            }
            imu.readGyro();

            // Add the Z axis reading to the total.
            total += imu.g.z;
        }
        this->gyroOffset_z = total / 2048*2;
    }


    // gyroIntegration
    float gyroIntegration(bool motorsState)
    {
        imu.readGyro();        
        this->update_dt();

        this->gyroz = ((float) (imu.g.z - (int16_t)this->gyroOffset_z))*GYRO_SCALE;
        if (motorsState){
            this->gyroAngle += (float)(this->gyroz * this->dt_time); // integrate when in motion
        }
            
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
    OdometryHandler(bool external_theta = false)
    {
        this->theta = 0;
        this->posx = 0;
        this->posy = 0;
        this->external_theta = external_theta;
    }

    void reset(float theta = 0){
        this->theta = theta;
        this->posx = 0;
        this->posy = 0;
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

        this->posy += (sin(theta + d_theta / 2) * (dx_1 + dx_2) / 2)/1000;
        this->posx += (cos(theta + d_theta / 2) * (dx_1 + dx_2) / 2)/1000;
        this->theta += d_theta;
    }
};

class PositionHandler
{
private:
    float theta;
    float posx;
    float posy;
    bool gyro_theta;
    OdometryHandler *odometryHandler;
    GyroHandler *gyroHandler;

public:
    PositionHandler(bool gyro_theta = false)
    {
        this->theta = 0;
        this->posx = 0;
        this->posy = 0;
        this->gyro_theta = gyro_theta;
        this->gyroHandler = new GyroHandler();
        this->odometryHandler = new OdometryHandler(gyro_theta);
    }

    void update(bool motorsState)
    {
        
        if (this->gyro_theta)
        {
            this->theta = gyroHandler->gyroIntegration(motorsState);
            odometryHandler->odometry(motorsState, this->theta);
        }
        else
        {
            odometryHandler->odometry(motorsState);
            this->theta = odometryHandler->theta;
        }
        this->posx = this->odometryHandler->posx;
        this->posy = this->odometryHandler->posy;
    }

    void reset(float x, float y, float angle){
        this->posx = x;
        this->posy = y;
        this->theta = angle;
        this->odometryHandler->reset(angle);
        this->gyroHandler->reset(angle);
    }

    float getx()
    {
        return this->posx;
    }

    float gety()
    {;
        return this->posy;
    }
    float getTheta()
    {
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

    if (current_index == path_length - 2)
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
    // Serial.print("vec1: anlge:");
    // Serial.print(atan2(v1.y, v1.x));
    // print_vec(v1);
    //     Serial.print("vec2: anlge:");
    // Serial.print(atan2(v2.y, v2.x));
    // print_vec(v2);
    float angle = max(angle1, angle2) - min(angle1, angle2);
    return (angle < PI) ? angle : (2 * PI - angle);
}

float next_point_controller(Vector2D point, Vector2D position, Vector2D direction_vector)
{
    // Path controller which return the next direction for the car
    Vector2D pointing_vector = point - position;
    float angle = inner_angle(direction_vector, pointing_vector);
    // Serial.print("      Inner Angle     ");
    // Serial.println(angle);
    // Serial.print("      pointing_vector     ");
    // print_vec(pointing_vector);

    return (is_clock_wise_angle(pointing_vector, direction_vector)) ? angle : -1*angle;
}

Line getSection(float path[][2], int index)
{
    Vector2D p1(path[index][0], path[index][1]);
    Vector2D p2(path[index + 1][0], path[index + 1][1]);
    return Line(p1, p2);
}

bool finished(Line section, Vector2D position, int index, float threshole = 0.05)
{
    if (index + 2 < path_length)
        return false;
    return about_to_pass_section(section, position, threshole);
}

enum CarState
{
    initiating,
    driving,
    rotating,
    smooth,
    done
};

class Car
{
private:
    Vector2D position;
    float direction;
    float velocity;
    float target_direction;
    float target_velocity;
    Vector2D last_position;
    unsigned long last_time;
    int state;

public:
    PositionHandler *positionHandler;
    int leftMotor, rightMotor;
    bool motorsState;
    PIDController *distance_pid_controller, *sharp_angle_pid_controller,*angle_pid_controller, *velocity_pid_controller;
    // Constructor
    Car(const Vector2D initial_position, const Vector2D initial_velocity)
    {
        this->distance_pid_controller =  new PIDController(0.1f, 0.01f, 0.0f, 0.1f);
        this->angle_pid_controller = new PIDController(160, 40, 0.0f, 0.1f);
        this->sharp_angle_pid_controller = new PIDController(160, 40, 0.0f, 0.1f);
        this->velocity_pid_controller = new PIDController(2800, 180, 0.0f, 0.1f);
        this->position = initial_position;
        this->direction = inner_angle(initial_velocity);
        this->velocity = initial_velocity.norm();
        this->positionHandler = new PositionHandler(false);
        this->leftMotor = 1;
        this->rightMotor = 1;
        this->state = CarState::driving;
    }

    Vector2D get_position()
    {
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
        Vector2D get_direction_vector()
    {
        return Vector2D(cos(this->direction), sin(this->direction));
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

    void update_position(int axis=rotateAxis::right)
    {
        motorsState = (leftMotor || rightMotor) == 0 ? 0 : 1;
        positionHandler->update(motorsState);
        this->leftMotor = 0;
        this->rightMotor = 0;
        this->direction = positionHandler->getTheta();
        float x = positionHandler->getx();
        float y = positionHandler->gety();
        unsigned long current_time = millis();
        float dt = (current_time - this->last_time) / 1000.0; // convert to seconds
        this->last_time = current_time;
        this->last_position = this->position;
        this->position = Vector2D(x, y);
        this->velocity = (this->last_position - this->position).norm() / dt;
    // print_vec(this->last_position - this->position);
    // Serial.println("Velocity");
        // int16_t leftCount = encoders.getCountsAndResetLeft();
        // int16_t RightCount = encoders.getCountsAndResetRight();
        // this->velocity = ((leftCount + RightCount) / 2) * encoder2dist / dt;
        float velocity_control_signal;
        if (this->target_velocity == 0)
            velocity_control_signal = 0;
            
        else
            velocity_control_signal = velocity_pid_controller->pid_control(this->target_velocity, this->velocity);
        if (velocity_control_signal < 0){
            velocity_control_signal = 0;
        }
        float angle_control_signal;
        if (this->state == CarState::rotating)
            angle_control_signal = this->sharp_angle_pid_controller->pid_control(this->target_direction, this->direction);
        else
            angle_control_signal = this->angle_pid_controller->pid_control(this->target_direction, this->direction);

        float axis_ratio = 1;
        switch (axis)
        {
        case rotateAxis::right:
            this->leftMotor = (int)-1*axis_ratio*angle_control_signal;
            this->rightMotor = (int)(1 - axis_ratio)*angle_control_signal;
            break;
        case rotateAxis::left:
            // Apply the control signal to the motors
            this->leftMotor = (int)-1 *(1- axis_ratio)* angle_control_signal;
            this->rightMotor = (int)axis_ratio*angle_control_signal;;

            break;

        default:
            // Apply the control signal to the motors
            this->leftMotor = (int)((-1) * angle_control_signal) / 2;
            this->rightMotor = (int)angle_control_signal / 2;

        
            break;
        }
    if (this->state == CarState::driving){
        // Serial.print("velocity_control_signal");
        // Serial.println(velocity_control_signal);
        
        this->leftMotor = (int)((velocity_control_signal + this->leftMotor) );
        this->rightMotor = (int)((velocity_control_signal+ this->rightMotor));
    }
        //     Serial.print("control signal:");
        // Serial.println(velocity_control_signal);
        // Serial.println(this->leftMotor);
        // Serial.println(this->rightMotor);

        motors.setLeftSpeed(this->leftMotor);
        motors.setRightSpeed(this->rightMotor);
    }

    void set_state(int new_state){
        this->state = new_state;
    }
    int get_state(){
        return this->state;
    }
};

// other intial states
/// Start ///
bool motorsState;
float Ts;
int axis = middle;

bool started = false;
Vector2D initial_pos(0, 0);
int section_index = 0;

Vector2D initial_velocity(0, 0.08);
float velocity = initial_velocity.norm();
Vector2D current_pos(0, 0);
Car car(initial_pos, initial_velocity);
Line current_section = getSection(path, section_index);
float new_direction;

void print_vec(Vector2D vec)
{
    Serial.print(vec.x);
    Serial.print(',');
    Serial.println(vec.y);
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


void rotate(){
    float epsilon = 0.05;
        Serial.println("debug: ----------rotating--------------");
        new_direction = next_point_controller(current_section.p2, car.get_position(), car.get_direction_vector());
        float target_angle = car.get_direction() + new_direction;
        Serial.print("New Rotation theta direction");
        Serial.println(target_angle);

        
        car.set_direction(target_angle);
        if (fmod(abs(car.get_direction() - target_angle), (float)(2 * PI)) < epsilon){
            car.set_state(CarState::driving);
            Serial.println("------------Debug: CarState is driving-----------");
        }
            
}

void smooth_motion(float pass_section_threshole = 0.05)
{
    if (car.get_state() == CarState::initiating){
        rotate();
        return;
    }
        
    // run simulation
    if (finished(current_section, car.get_position(), section_index))
        car.set_state(CarState::done);

    // find next section
    if (about_to_pass_section(current_section, car.get_position(), pass_section_threshole) && ((section_index + 2) < path_length))
    {
        section_index++;
        car.set_state(CarState::rotating);
        current_section = getSection(path, section_index);
        Serial.println("------------Debug: Next Target point-----------");
        print_vec(current_section.p2);
    }
    new_direction = next_point_controller(current_section.p2, car.get_position(), car.get_direction_vector());
    Serial.print("New direction:");
    Serial.println(new_direction);
    if ((car.get_state() == CarState::initiating)  && (!polygon))
        rotate();
    if ((!polygon)){
        car.set_direction(car.get_direction() + new_direction);
    }
        
}

void polygon_motion()
{
    // run simulation
    // set velocity to zero and roatate car
    if (car.get_state() == CarState::initiating)
        rotate();
    if (car.get_state() == CarState::driving)
    {
        smooth_motion(0);
    }
    if (car.get_state() == CarState::rotating)
    {
        rotate();
    }
}

void get_circle_path(float radius)
{
    float angle_increasment = (2 * PI) / ((float) (path_length-1) );
    Serial.print("Angle inc");
    Serial.println(angle_increasment);
    float x, y;
    float angle = 0;
    int i;
    for (i = 0; i < path_length; i++)
    {
        x = radius * (cos(angle)-1);
        y = radius * sin(angle);
        angle += angle_increasment;
        path[i][0] = x;
        path[i][1] = y;
        Serial.println("Circle: ");
        print_vec(Vector2D(path[i][0], path[i][1]));
    }
    for (i = i; i < PATH_MAX_LENGTH; i++)
    {
        path[i][0] = 0;
        path[i][1] = 0;
    }
}

void get_square_path(float edge_length)
{
    path[0][0] = 0;
    path[0][1] = 0;

    path[1][0] = 0;
    path[1][1] = edge_length;

    path[2][0] = -1*edge_length;
    path[2][1] = edge_length;

    path[3][0] = -1*edge_length;
    path[3][1] = 0;

    path[4][0] = 0;
    path[4][1] = 0;

    path_length = 5;
    Serial.println("Path:");
    print_vec(Vector2D(path[0][0], path[0][1]));
    print_vec(Vector2D(path[1][0], path[1][1]));
    print_vec(Vector2D(path[2][0], path[2][1]));
    print_vec(Vector2D(path[3][0], path[3][1]));
    print_vec(Vector2D(path[4][0], path[4][1]));


}

void get_shape_path(int shape){
    switch(shape){
        case (PathShape::squareShape):
            get_square_path(0.2);
            break;
        default:
            get_circle_path(0.2);
    }
}


void setup()
{
    // initialize serial:
    Serial.begin(9600);
    buttonB.waitForButton();
    get_shape_path(path_shape);
    Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  car.set_state(CarState::initiating);
    // initialize serial:
    current_section = getSection(path, section_index);
    car.set_velocity(velocity);
    car.positionHandler->reset(0, 0, inner_angle(car.get_velocity_vector()));
    Ts = 0.01;
    
    Serial.print("------------Debug: Next Target point-----------     ");
    print_vec(current_section.p2);
}

// //uncommen this for position handler checking
// void loop(){
//     // Serial.println("Debug: update");
//     car.positionHandler->update(true);
//     Serial.print("x: ");
//     Serial.print(car.positionHandler->getx());
//     Serial.print("y: ");
//     Serial.print(car.positionHandler->gety());
//     Serial.println("theta: ");
//     Serial.println(car.positionHandler->getTheta());
// delay(100);
// }

//uncooment this for to test car
// void loop(){
//     car.set_velocity(0);
//     car.set_direction(2*PI);
//     car.update_position(rotateAxis::middle);
//     // Serial.println("Debug: update");
//     car.positionHandler->update(true);
//     Serial.print("x: ");
//     Serial.println(car.positionHandler->getx());
//     Serial.print("y: ");
//     Serial.println(car.positionHandler->gety());
//     Serial.print("theta: ");
//     Serial.println(car.positionHandler->getTheta());
//     Serial.print("Velocity: ");
//     Serial.println(car.get_velocity());
// delay(100);
// }

// //uncomment this for path controll
bool printed_done = false;
void loop()
{
    if (car.get_state() != CarState::done)
    {

            if (polygon)
            {
                polygon_motion();
                car.update_position(rotateAxis::left);
            }
            else
            {
                smooth_motion();
                car.update_position(rotateAxis::middle);
            }
        // car.positionHandler->update(1);
        // Serial.print("Position: ");
        // print_vec(car.get_position());
        Serial.print("theta: ");
        Serial.println(car.positionHandler->getTheta());
        // Serial.print("theta vector: ");
        // print_vec(car.get_direction_vector());
        // Serial.print("Velocity vector: ");
        // print_vec(car.get_velocity_vector());
    }
    else
    {
     if (!printed_done){
        motors.setLeftSpeed(0);
        motors.setRightSpeed(0);   
        Serial.println("done");
        printed_done = true;
     }
       
    }
  if (buttonB.isPressed())
  {
    car.set_state(CarState::done);
    car.set_velocity(0);
  }


}

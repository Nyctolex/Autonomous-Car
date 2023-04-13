#include "PIDController.h""

PID_Contrller::PID_Contrller(double kp, double ki, double kd, double target_value, double target_min_value, double target_max_value){
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
void PID_Contrller::reset_controller(){
  this->cumulative_error = 0;
  this->inetgrated_error = 0;
  this->previous_error = 0;
  this->prev_time = millis();
}

double PID_Contrller::next(double sensor_output){
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

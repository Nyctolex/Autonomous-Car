class PIDController {
public:
    PIDController(float kp, float ki, float kd) 
        : Kp(kp), Ki(ki), Kd(kd), integral(0), prev_error(0), prev_derivative(0), filtered_derivative(0) {}

    float calculate(float setpoint, float process_variable) {
        // Calculate time since last sample
        unsigned long current_time = millis();
        float dt = (current_time - last_time) / 1000.0; // convert to seconds
        last_time = current_time;

        // Update Ts
        Ts = dt;

        // Calculate error
        float error = setpoint - process_variable;

        // Calculate integral term
        integral += error * Ts;

        // Calculate derivative term with FIR filter
        derivative = (error - prev_error) / Ts;
        filtered_derivative = alpha * derivative + (1 - alpha) * prev_derivative;

        // Calculate control signal
        float control_signal = Kp * error + Ki * integral + Kd * filtered_derivative;

        // Store previous error and derivative for next iteration
        prev_error = error;
        prev_derivative = filtered_derivative;

        return control_signal;
    }

private:
    const float Kp;
    const float Ki;
    const float Kd;
    const float alpha = 0.1;
    float integral;
    float prev_error;
    float prev_derivative;
    float derivative;
    float filtered_derivative;
    float Ts = 0.01;
    unsigned long last_time = millis();
};

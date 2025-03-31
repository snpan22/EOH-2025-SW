class PIDController {
    public:
        PIDController(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            previousError = 0;
            integral = 0;
        }
    
        float compute(float setpoint, float measuredValue) {
            float error = setpoint - measuredValue;
            integral += error;
            float derivative = error - previousError;
            previousError = error;
    
            return (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
    
        void setTunings(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }
    
        void reset() {
            integral = 0;
            previousError = 0;
        }
    
    private:
        float Kp, Ki, Kd;
        float previousError, integral;
    };
  
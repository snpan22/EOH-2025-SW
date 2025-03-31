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
            unsigned long currentTime = millis();  
            float deltaTime = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds
            
        
            float error = setpoint - measuredValue;
            integral += error * deltaTime;  // Integral considers time step
            float derivative = (error - previousError) / deltaTime;  // Derivative considers rate of change
        
            previousError = error;
            lastTime = currentTime;  // Update timestamp
        
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
        float lastTime;
    };
  
class PIDController {
    public:
        PIDController(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            previousError = 0;
            integral = 0;
            lastTime = 0;
        }
    
        float compute(float setpoint, float measuredValue) {
            uint32_t currentTime = millis();
            // Serial.println("currentTime: ");
            // Serial.println(currentTime);  
            float deltaTime = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds
            // Serial.println("deltatime: ");
            // Serial.println(deltaTime);
        
            float error = setpoint - measuredValue;
            integral += error * deltaTime;  // Integral considers time step
            float derivative = (error - previousError) / deltaTime;  // Derivative considers rate of change
            Serial.print("derivative");
            Serial.println(derivative);
            
            // Serial.print("error:  ");
            // Serial.println(error);
            // Serial.print("integral:  ");
            // Serial.println(integral);
            // Serial.print('d:  ');
            // Serial.println(derivative);
        
            previousError = error;
            lastTime = currentTime;  // Update timestamp
            // Serial.println("lasttime: ");
            // Serial.println(lastTime);
            // Serial.println("LastTime: ");
            // Serial.println(lastTime);

            return (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
    
        void setTunings(float kp, float ki, float kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }
        float get_kp(){
            return Kp;
        }
    
        float get_ki(){
            return Ki;
        }

        float get_kd(){
            return Kd;
        }
        float get_integral(){
            return integral;
        }

        float get_previousError(){
            return previousError;
        }
        
        void reset() {
            integral = 0;
            previousError = 0;
            lastTime = millis();
        }
    
    private:
        float Kp, Ki, Kd;
        float previousError, integral;
        uint32_t lastTime;
    };
  
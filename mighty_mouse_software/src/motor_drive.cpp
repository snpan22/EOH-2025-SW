#include <motor_drive.h>

//analog  values on teensy range from 0 to 1023
void drive_motor(int pwm, MotorPins motor){
if (pwm < 0) { //checks if motor should move backwards
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  } else { //else move motor forward 
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  }
  analogWrite(motor.en, abs(pwm));
}
/*
 *Directions:
 * 1 = left
 * 2 = right
 * 0 = straight
 * 180 = 180 turn
 * 
 * pwm values will be
*/
void direction_drive(int direction, int pwm_ml, int pwm_mr, Robot mouse){
    //straight:positive pwm value to both motors. should be same
    if(direction == 0){
        drive_motor(pwm_ml, mouse.left);
        drive_motor(pwm_mr, mouse.right);
    }
    //reverse left motor to make sharp turn left
    //!currently set to sharp turn. may  need to experiment by making turn less sharp
    //!do this by making one motor x% of the other (slow it down) instead of negative (Reverse)
    else if(direction == 1){
        drive_motor(-pwm_ml, mouse.left);
        drive_motor(pwm_mr, mouse.right);
    }
    //reverse right motor to make sharp turn right
    else if(direction == 1){
        drive_motor(pwm_ml, mouse.left);
        drive_motor(-pwm_mr, mouse.right);
    }
    else if (direction == 180){
        //how to turn 180?
        
    }

}
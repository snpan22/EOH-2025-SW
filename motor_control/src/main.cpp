#include <Arduino.h>
#include <Wire.h>
#include <motor_drive.h>

// #define TEST_MOTORS // use when you want to run motors (hardcoded program)
#define SERIAL      // use when you want to run maze solving program (communicate with Raspberry Pi)

MotorPins left{ml_en, ml_in_1, ml_in_2, ml_outa, ml_outb};
MotorPins right{mr_en, mr_in_1, mr_in_2, mr_outa, mr_outb};

Robot mouse{left, right};

void setup()
{
  // motor control pins
  pinMode(mr_in_1, OUTPUT);
  pinMode(mr_in_2, OUTPUT);
  pinMode(ml_in_1, OUTPUT);
  pinMode(ml_in_2, OUTPUT);
  pinMode(mr_en, OUTPUT);
  pinMode(ml_en, OUTPUT);

  // motor encoder pins
  pinMode(mr_outa, INPUT);
  pinMode(mr_outb, INPUT);
  pinMode(ml_outa, INPUT);
  pinMode(ml_outb, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:

  // analog  values on teensy range from 0 to 1023
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    int dir_left, pwm_left, dir_right, pwm_right;
    sscanf(command.c_str(), "%d,%d,%d,%d", &dir_left, &pwm_left, &dir_right, &pwm_right);
    if(dir_left ==0){
      analogWrite(ml_en, 0);
      digitalWrite(ml_in_1, 0); 
      digitalWrite(ml_in_2, 0);
    } 
    if(dir_right == 0){
      analogWrite(mr_en, 0);
      digitalWrite(mr_in_1, 0); 
      digitalWrite(mr_in_2, 0);  
    }
    if(dir_left !=0 && dir_right != 0){

      //! left motor
      //! in1 LOW, in2 HIGH--> Forward
      //! in1 HIGH, in2 LOW-->Backwards
      analogWrite(ml_en, pwm_left);
      digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
      digitalWrite(ml_in_2, dir_left == 1); // 
      // analogWrite(ml_en, 100);
      // digitalWrite(ml_in_1, LOW);
      // digitalWrite(ml_in_2, HIGH);
      //! RIGHT motor
      //! in1 HIGH, in2 LOW--> Forward
      //! in1 LOW, in2 HIGH-->Backwards
      analogWrite(mr_en, pwm_right);
      digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
      digitalWrite(mr_in_2, dir_right == -1); // 
      // analogWrite(mr_en, 100);
      // digitalWrite(mr_in_1, HIGH);
      // digitalWrite(mr_in_2, LOW);
    }
  }
}

  // digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);



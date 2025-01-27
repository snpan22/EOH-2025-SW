#include <Arduino.h>
#include <Wire.h>
#include <motor_drive.h>

#define TEST_MOTORS //use when you want to run motors (hardcoded program)
#define SERIAL     //use when you want to run maze solving program (communicate with Raspberry Pi)

MotorPins left {ml_en, ml_in_1, ml_in_2, ml_outa, ml_outb};
MotorPins right {mr_en, mr_in_1, mr_in_2, mr_outa, mr_outb};

Robot mouse{left, right};



void setup() {
  // motor control pins
  pinMode(mr_in_1, OUTPUT);
  pinMode(mr_in_2, OUTPUT);
  pinMode(ml_in_1, OUTPUT);
  pinMode(ml_in_2, OUTPUT);
  pinMode(mr_en, OUTPUT);
  pinMode(ml_en, OUTPUT);

  //motor encoder pins
  pinMode(mr_outa, INPUT);
  pinMode(mr_outb, INPUT);
  pinMode(ml_outa, INPUT);
  pinMode(ml_outb, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  //analog  values on teensy range from 0 to 1023

// #ifdef TEST_MOTORS

// #endif
}


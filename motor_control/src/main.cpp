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
#ifdef TEST_MOTORS
  // if(Serial.available()){
  //   char val[1];
  //   Serial.readBytes(val, 1);

  //   if(val[0] == 1){
  //     digitalWrite(mr_in_1, LOW);
  //     digitalWrite(mr_in_2, HIGH);
  //     analogWrite(mr_en, 255);
  //     delay(2000); // Run for 2 seconds
  //     digitalWrite(mr_in_1, HIGH);
  //     digitalWrite(mr_in_2, LOW);
  //     analogWrite(mr_en, 255);
  //     delay(2000); // Run for 2 seconds
  //   }
  //   else if(val[0] == 2){
  //     digitalWrite(ml_in_1, LOW);
  //     digitalWrite(ml_in_2, HIGH);
  //     analogWrite(ml_en, 255);
  //     delay(2000); // Run for 2 seconds
  //     digitalWrite(ml_in_1, HIGH);
  //     digitalWrite(ml_in_2, LOW);
  //     analogWrite(ml_en, 255);
  //     delay(2000); // Run for 2 seconds
  //   }

  // }
  digitalWrite(mr_in_1, LOW);
  digitalWrite(mr_in_2, HIGH);
  analogWrite(mr_en, 255);
  delay(2000); // Run for 2 seconds
  digitalWrite(mr_in_1, HIGH);
  digitalWrite(mr_in_2, LOW);
  analogWrite(mr_en, 255);
  delay(2000); // Run for 2 seconds

  digitalWrite(ml_in_1, LOW);
  digitalWrite(ml_in_2, HIGH);
  analogWrite(ml_en, 255);
  delay(2000); // Run for 2 seconds
  digitalWrite(ml_in_1, HIGH);
  digitalWrite(ml_in_2, LOW);
  analogWrite(ml_en, 255);
  delay(2000); // Run for 2 seconds

#endif

digitalWrite(LED_BUILTIN, HIGH);
delay(500);
digitalWrite(LED_BUILTIN, LOW);
delay(500);
if (Serial.available()) {
  char val = Serial.read();  // Read the incoming byte
  
  if (val == 1) {
    // Toggle the built-in LED on the Teensy
    // digitalWrite(13, HIGH);  // Turn on the LED
    // delay(500);              // Keep it on for 500ms
    // digitalWrite(13, LOW);   // Turn off the LED
    // delay(500);              // Keep it off for 500ms
    digitalWrite(ml_in_1, LOW);
    digitalWrite(ml_in_2, HIGH);
    analogWrite(ml_en, 200);
    // delay(2000); // Run for 2 seconds

    // right is good i think. left is sus
    digitalWrite(mr_in_2, LOW);
    digitalWrite(mr_in_1, HIGH);
    analogWrite(mr_en,200);
    delay(200); // Run for 2 seconds
    digitalWrite(mr_in_2, LOW);
    digitalWrite(mr_in_1, LOW);
    digitalWrite(ml_in_1, LOW);
    digitalWrite(ml_in_2, LOW);

    // analogWrite(mr_en,0);
    // analogWrite(ml_en, 0);
    // Send confirmation back to Raspberry Pi
    // Serial.write(1);         // Send '1' to indicate the LED was toggled
  }
}

  // digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);
}

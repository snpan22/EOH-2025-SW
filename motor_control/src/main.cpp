#include <Arduino.h>
#include <Wire.h>
#include <motor_drive.h>
#include <PIDController.h>

// const byte ledPin = 13;
// const byte interruptPin = 2;
// volatile byte state = LOW;

// void setup() {
//   pinMode(ledPin, OUTPUT);
//   pinMode(interruptPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
// }

// void loop() {
//   digitalWrite(ledPin, state);
// }

// void blink() {
//   state = !state;
// }

// // #define TEST_MOTORS // use when you want to run motors (hardcoded program)
#define SERIAL      // use when you want to run maze solving program (communicate with Raspberry Pi)

MotorPins left{ml_en, ml_in_1, ml_in_2, ml_outa, ml_outb};
MotorPins right{mr_en, mr_in_1, mr_in_2, mr_outa, mr_outb};

Robot mouse{left, right};
volatile int encoderPosRight = 0;  // Track the encoder position
volatile int encoderPosLeft = 0;  // Track the encoder position
void encoderISR_right();
void encoderISR_left();

float kpl = 0.5;
float kil = 1.;
float kdl = 0.01;
float ul = 0.;

PIDController controller_left(kpl,kil,kdl);

float kpr = 0.5;
float kir = 1.;
float kdr = 0.01;
float ur = 0.;

PIDController controller_right(kpr,kir,kdr);

int setpoint = 100;

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
  attachInterrupt(digitalPinToInterrupt(mr_outa), encoderISR_right, CHANGE);  // Trigger on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(ml_outa), encoderISR_left, CHANGE);  // Trigger on both rising and falling edges
  pinMode(LED_BUILTIN, OUTPUT);
  //faster baud rate :115200 
  Serial.begin(115200);
}
void loop()
{
  // put your main code here, to run repeatedly:

    int dir_right = 1;
    int dir_left = 1;


    ul = controller_left.compute(setpoint,encoderPosLeft);
    ur = controller_right.compute(setpoint,encoderPosRight);

    



    analogWrite(mr_en, ur);

    digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, dir_right == -1); //

    analogWrite(ml_en, ul);

    digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, dir_left == 1); // 



    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print("Right position: "); 
    Serial.print(encoderPosRight);  // Print encoder position for debugging
    Serial.print("  ");
    Serial.print("Left position: ");
    Serial.print(encoderPosLeft);  // Print encoder position for debugging
    Serial.println("  Next Line");

    // Serial.println(encoderPos);  // Print encoder position for debugging

    delay(100);
  // analog  values on teensy range from 0 to 1023
  // if (Serial.available()) {
  //   String command = Serial.readStringUntil('\n');
  //   int dir_left, pwm_left, dir_right, pwm_right;
  //   sscanf(command.c_str(), "%d,%d,%d,%d", &dir_left, &pwm_left, &dir_right, &pwm_right);
  //   if(dir_left ==0){
  //     analogWrite(ml_en, 0);
  //     digitalWrite(ml_in_1, 0); 
  //     digitalWrite(ml_in_2, 0);
  //   } 
  //   if(dir_right == 0){
  //     analogWrite(mr_en, 0);
  //     digitalWrite(mr_in_1, 0); 
  //     digitalWrite(mr_in_2, 0);  
  //   }
  //   if(dir_left !=0 && dir_right != 0){

  //     //! left motor
  //     //! in1 LOW, in2 HIGH--> Forward
  //     //! in1 HIGH, in2 LOW-->Backwards
  //     analogWrite(ml_en, pwm_left);
  //     digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
  //     digitalWrite(ml_in_2, dir_left == 1); // 
  //     // analogWrite(ml_en, 100);
  //     // digitalWrite(ml_in_1, LOW);
  //     // digitalWrite(ml_in_2, HIGH);
  //     //! RIGHT motor
  //     //! in1 HIGH, in2 LOW--> Forward
  //     //! in1 LOW, in2 HIGH-->Backwards
  //     analogWrite(mr_en, pwm_right);
  //     digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
  //     digitalWrite(mr_in_2, dir_right == -1); // 
  //     // analogWrite(mr_en, 100);
  //     // digitalWrite(mr_in_1, HIGH);
  //     // digitalWrite(mr_in_2, LOW);
  //   }
  // }
}
void encoderISR_right() {

  // This function will be called every time the encoder pin A changes state
  int stateA = digitalRead(mr_outa);
  int stateB = digitalRead(mr_outb);
  
  // Update the encoder position based on the direction of rotation
  if (stateA == stateB) {
      encoderPosRight++;
  } else {
      encoderPosRight--;
  }
}

void encoderISR_left() {

  // This function will be called every time the encoder pin A changes state
  int stateA = digitalRead(ml_outa);
  int stateB = digitalRead(ml_outb);
  
  // Update the encoder position based on the direction of rotation
  if (stateA == stateB) {
      encoderPosLeft--;
  } else {
      encoderPosLeft++;
  }
}


//   // digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
//   // delay(500);
//   // digitalWrite(LED_BUILTIN, LOW);
//   // delay(500);




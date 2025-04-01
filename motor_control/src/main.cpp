#include <Arduino.h>
#include <Wire.h>
#include <motor_drive.h>
#include <PIDController.h>
#include <algorithm>

// #define STRAIGHT
// #define TURN_LEFT
// #define TURN_RIGHT
// #define TURN_AROUND

#define ALL_TURNS

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

float kpl = 0.1;
float kil = 0.;
float kdl = 0.0;
float ul = 0.;

PIDController controller_left(kpl,kil,kdl);

float kpr = 0.1;
float kir = 0.;
float kdr = 0.0;
float ur = 0.;

PIDController controller_right(kpr,kir,kdr);

int setpoint = 315;
int setpointIndex = 0;
int numSetpoints = 2;

int turncount_left = 0;
int turncount_right = 0;
int turncount_around = 0;


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
  delay(5000);
}
void loop()
{
  int dir_right = 1;
  int dir_left = 1;
  // put your main code here, to run repeatedly:
#ifdef STRAIGHT
  

    if (encoderPosLeft != 0 && encoderPosRight != 0){
      
      // left: .13,.00425, .001
      // right: .1, .002, .001

      //for 750:
      //left: 0.15,0.01,0.001
      //right: 0.15, 0.0040, 0.001


      //for 500:
      // left: 0.275,0.01,0.001
      // right: 0.23, 0.0040, 0.001)

      controller_left.setTunings(0.540,0.01,0.001);
      controller_right.setTunings(0.43, 0.0040, 0.001);
    } 
    else{
      controller_left.setTunings(.35,0.0,0.0);
      controller_right.setTunings(.52, 0.0, 0.0);
    }
    ul = controller_left.compute(setpoint,encoderPosLeft);
    ur = controller_right.compute(setpoint,encoderPosRight);
    int pwr_r =  std::min(static_cast<int>(abs(ur)), 150);
    int pwr_l =  std::min(static_cast<int>(abs(ul)), 255);
    analogWrite(mr_en, pwr_r);

    digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, dir_right == -1); //

    analogWrite(ml_en, pwr_l);

    digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, dir_left == 1); // 



    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print("| ul: ");
    Serial.print(ul);
    Serial.print(" | ur: ");
    Serial.print(ur);
    Serial.print("| pwr_l: ");
    Serial.print(pwr_l);
    Serial.print(" | pwr_r: ");
    Serial.print(pwr_r);
    Serial.print("Right position: "); 
    Serial.print(encoderPosRight);  // Print encoder position for debugging
    Serial.print("  ");
    Serial.print("Left position: ");
    Serial.println(encoderPosLeft);  // Print encoder position for debugging

    // Serial.println(encoderPos);  // Print encoder position for debugging


    if (((encoderPosLeft - setpoint > 10) || (encoderPosRight - setpoint > 0)) && (setpointIndex < numSetpoints)) {
      Serial.print("Setpoint reached! Moving to setpoint index:");
      Serial.println(setpointIndex);
      encoderPosLeft = 0; // Reset encoder positions
      encoderPosRight = 0;

      setpointIndex++; // Move to next setpoint

    }
  #endif
  // digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
  // digitalWrite(mr_in_2, dir_right == -1); //

  // analogWrite(ml_en, pwr_l);

  // digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
  // digitalWrite(ml_in_2, dir_left == 1); // 
  #ifdef TURN_RIGHT
  if(turncount_right < 10){
    analogWrite(mr_en, 255);

    digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, dir_right == -1); //
    analogWrite(ml_en, 210);

    digitalWrite(ml_in_1, dir_left == 1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, dir_left == -1); // 
    turncount_right++;
  }
  else{
    digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, LOW); //

    digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, LOW); //
    // turncount_right = 0;
  }
  #endif

  #ifdef TURN_LEFT
  if(turncount_left < 10){
    analogWrite(mr_en, 195);

    digitalWrite(mr_in_1, dir_right == -1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, dir_right == 1); //

    analogWrite(ml_en, 255);

    digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, dir_left == 1); // 
    turncount_left++;
  }
  else{
    digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, LOW); //

    digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, LOW); //
  }
  #endif


  #ifdef TURN_AROUND
  if(turncount_around < 15){
    analogWrite(mr_en, 255);

    digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, dir_right == -1); //

    analogWrite(ml_en, 255);

    digitalWrite(ml_in_1, dir_left == 1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, dir_left == -1); // 
    turncount_around++;
  }
  else{
    digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, LOW); //

    digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, LOW); //
  }
  #endif
    delay(10);
  
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



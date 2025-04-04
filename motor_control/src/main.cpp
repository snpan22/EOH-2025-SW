#include <Arduino.h>
#include <Wire.h>
#include <motor_drive.h>
#include <PIDController.h>
#include <algorithm>

// #define STRAIGHT
// #define TURN_LEFT
// #define TURN_RIGHT
// #define TURN_AROUND

#define PROGRAM

// # define TEST_IR
uint8_t IR_1 = 33;
// // #define TEST_MOTORS // use when you want to run motors (hardcoded program)

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

//!static variables only available in this file... change to non static if passing thru functions
static int turncount_left = 0;
static int turncount_right = 0;
static int turncount_around = 0;

static uint8_t lastcommand = 0;
static uint8_t command = 0;

int pwr_r = 0;
int pwr_l = 0;

int dir_right = 1;
int dir_left = 1;


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

  pinMode(IR_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(mr_outa), encoderISR_right, CHANGE);  // Trigger on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(ml_outa), encoderISR_left, CHANGE);  // Trigger on both rising and falling edges
  pinMode(LED_BUILTIN, OUTPUT);
  //faster baud rate :115200 
  Serial.begin(115200);
  // delay(5000);
  while (!Serial);

    // Flush the serial buffer
  while (Serial.available()) {
      Serial.read();  // Discard any leftover bytes
  }
}
void loop()
{
  #ifdef TEST_IR
  int ir1_value = analogRead(IR_1);
  Serial.println(ir1_value);
 #endif
  // put your main code here, to run repeatedly:
#ifdef PROGRAM
// digitalWrite(LED_BUILTIN, LOW);
  turncount_right = 0;
  turncount_left = 0;
  turncount_around = 0;
  encoderPosRight = 0;  // Track the encoder position
  encoderPosLeft = 0;  // Track the encoder position
  pwr_l = 0;
  pwr_r = 0;
  ur = 0;
  ul = 0;
  controller_left.reset();
  controller_right.reset();
  if (Serial.available()) {
    command = Serial.read();  // Read single byte
    // Serial.print("teensy read: ");
    // Serial.println(command);
    if (command!=lastcommand) {
      Serial.println("resetting global variable");
      turncount_right = 0;
      turncount_left = 0;
      turncount_around = 0;
      encoderPosRight = 0;  // Track the encoder position
      encoderPosLeft = 0;  // Track the encoder position
      pwr_l = 0;
      pwr_r = 0;
      ur = 0;
      ul = 0;
      controller_left.reset();
      controller_right.reset();
    } 
    switch (command) {
        case 0:
            // stop
            digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(mr_in_2, LOW); //
        
            digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(ml_in_2, LOW); //
            break;
        case 1:
            // straight
            Serial.print("previouserror left: ");
            Serial.println(controller_left.get_previousError());
            Serial.print("previouserror right: ");
            Serial.println(controller_right.get_previousError());
            Serial.print("integral left: ");
            Serial.println(controller_left.get_integral());
            Serial.print("integral right: ");
            Serial.println(controller_right.get_integral());
            while(((encoderPosLeft - setpoint < 10) && (encoderPosRight - setpoint < 0))){
              Serial.println("entered going straight while loop");
              Serial.print("encoderpos left: ");
              Serial.print(encoderPosLeft);
              Serial.print("| encoderpos right: ");
              Serial.print(encoderPosRight);

              if (encoderPosLeft != 0 && encoderPosRight != 0){
        
                // left: .13,.00425, .001
                // right: .1, .002, .001
          
                //for 750:
                //left: 0.15,0.01,0.001
                //right: 0.15, 0.0040, 0.001
          
          
                //for 500:
                // left: 0.275,0.01,0.001
                // right: 0.23, 0.0040, 0.001)
          
                // controller_left.setTunings(0.625,0.205,0.0025);
                controller_left.setTunings(0.500,0.205,0.0025);
                // controller_right.setTunings(0.475, 0.0060, 0.001);

                controller_right.setTunings(0.500, 0.205, 0.0001);
              } 
              else{
                controller_left.setTunings(.5,0.0,0.0);
                controller_right.setTunings(.5, 0.0, 0.0);
                
              }
              
              // Serial.print("| left kp: ");
              // Serial.print(controller_left.get_kp());
              // Serial.print("| left ki: ");
              // Serial.print(controller_left.get_ki());
              // Serial.print("| left kd: ");
              // Serial.print(controller_left.get_kd());

              // Serial.print("| right kp: ");
              // Serial.print(controller_right.get_kp());
              // Serial.print("| right ki: ");
              // Serial.print(controller_right.get_ki());
              // Serial.print("|right kd: ");
              // Serial.print(controller_right.get_kd());
              
              // ul = 130;
              // ur = 100;
              
              ul = controller_left.compute(setpoint,encoderPosLeft);
              ur = controller_right.compute(setpoint,encoderPosRight);

              Serial.print("| ul: ");
              Serial.print(ul);
              Serial.print("| ur: ");
              Serial.println(ur);

              Serial.print("pwr_l: ");
              Serial.print(pwr_l);
              Serial.print("| pwr_r: ");
              Serial.println(pwr_r);

              //! messed with these power caps. different from straight test. tese dont work either
              pwr_r =  std::min(static_cast<int>(abs(ur)), 200);
              pwr_l =  std::min(static_cast<int>(abs(ul)), 200);

              Serial.print("| pwr_l: ");
              Serial.print(pwr_l);
              Serial.print("| pwr_r: ");
              Serial.println(pwr_r);
              analogWrite(mr_en, pwr_r);
          
              digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(mr_in_2, dir_right == -1); //
          
              analogWrite(ml_en, pwr_l);
          
              digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(ml_in_2, dir_left == 1); //
              
              // if (((encoderPosLeft - setpoint > 10) || (encoderPosRight - setpoint > 0))) {
              //   //!Tried && instead of ||. Setpoint never reached, just goes to next command
              //   digitalWrite(LED_BUILTIN, HIGH);
                
              // }
              delay(100);

            }
            Serial.println("resetting pid stuff");
            encoderPosRight = 0;  // Track the encoder position
            encoderPosLeft = 0;  // Track the encoder position
            pwr_l = 0;
            pwr_r = 0;
            ur = 0;
            ul = 0;
            controller_left.reset();
            controller_right.reset();
        
            break;
        case 2:
            // Turn right
            // 1 step, right 255, left 210 previous version
            while(turncount_right < 4){
              analogWrite(mr_en, 140);
          
              digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(mr_in_2, dir_right == -1); //
              analogWrite(ml_en, 140);
          
              digitalWrite(ml_in_1, dir_left == 1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(ml_in_2, dir_left == -1); // 
              turncount_right++;
              delay(100);
            }
            // else{
            digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(mr_in_2, LOW); //
        
            digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(ml_in_2, LOW); //
            //   // turncount_right = 0;
            // }
            break;
        case 3:
            // Turn left
            // 1 step, right 225, left 255 previous version
            while(turncount_left < 2){
              analogWrite(mr_en, 225);
          
              digitalWrite(mr_in_1, dir_right == -1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(mr_in_2, dir_right == 1); //
          
              analogWrite(ml_en, 255);
          
              digitalWrite(ml_in_1, dir_left == -1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(ml_in_2, dir_left == 1); // 
              turncount_left++;
              delay(100);
            }
            // else{
            digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(mr_in_2, LOW); //
        
            digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(ml_in_2, LOW); //
            // }
            break;
        case 4:
            //turn around
            while(turncount_around < 2){
              analogWrite(mr_en, 250);
          
              digitalWrite(mr_in_1, dir_right == 1); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(mr_in_2, dir_right == -1); //
          
              analogWrite(ml_en, 200);
          
              digitalWrite(ml_in_1, dir_left == 1); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
              digitalWrite(ml_in_2, dir_left == -1); // 
              turncount_around++;
              delay(100);
            }
            // else{
            digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(mr_in_2, LOW); //
        
            digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(ml_in_2, LOW); //
            // }
            break;
          
        default:
            // stop
            digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(mr_in_2, LOW); //
        
            digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
            digitalWrite(ml_in_2, LOW); //
            break;
    }
    lastcommand = command; // Store last received command

  }
  else{
    // stop
    digitalWrite(mr_in_1, LOW); //when dir_left == 1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(mr_in_2, LOW); //

    digitalWrite(ml_in_1, LOW); //when dir_left == -1, TRUE = HIGH, else FALSE = LOW
    digitalWrite(ml_in_2, LOW); //
  }
#endif
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

  delay(100);
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


# include <pins.h>
# include <Arduino.h>
#pragma once


void drive_motor(int pwm, MotorPins motor);
void direction_drive(int direction, int pwm_ml, int pwm_mr, Robot mouse);

#pragma once
#include <Arduino.h>


struct MotorPins{
    int en;
    int in1;
    int in2;
    int outa;
    int outb;
};

struct Robot{
    MotorPins left;
    MotorPins right;
};
//right motor input
constexpr int mr_in_1 =23;
constexpr int mr_in_2 =22;
constexpr int mr_en = 4;

//right motor output (encoder)
constexpr int mr_outa = 1;
constexpr int mr_outb = 2;

//left motor input
constexpr int ml_in_1 = 19;
constexpr int ml_in_2 = 18;
constexpr int ml_en = 3;

//left motor output (encoder)
constexpr int ml_outa = 5;
constexpr int ml_outb = 6;

constexpr uint8_t IR_RECEIVE_1 = A14;
constexpr uint8_t IR_RECEIVE_2 = A15;
constexpr uint8_t IR_RECEIVE_3 = A6;
constexpr uint8_t IR_RECEIVE_4 = A7;



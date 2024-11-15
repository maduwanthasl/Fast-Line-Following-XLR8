#pragma once

#include "math.h"
#include "Arduino.h"

//*******************************************************Motors****************************************************************

const int LeftEncoderPin1 = 35;
const int LeftEncoderPin2 = 45;

const int RightEncoderPin1 = 48;
const int RightEncoderPin2 = 47;

// motor controller 1
const int LEFT_MOTOR_PWM = 42;
const int LEFT_MOTOR_IN1 = 41;
const int LEFT_MOTOR_IN2 = 40;
const int RIGHT_MOTOR_IN2 = 39;
const int RIGHT_MOTOR_IN1 = 37;
const int RIGHT_MOTOR_PWM = 36;

const int PULSES_PER_ROTATION = 1495; // encoder pulse count for rotation, all have same values for 20 rotations (1494, 1495, 1496, 1497 averages)
const int WHEEL_DIAMETER_BACK = 65;
const int WHEEL_DIAMETER_FRONT = 63;
const float WHEEL_GAP = 189.5; // distance between the wheels in mm
const float MM_PER_ROTATION_BACK = WHEEL_DIAMETER_BACK * PI;
const float MM_PER_ROTATION_FRONT = WHEEL_DIAMETER_FRONT * PI;
const float DEG_PER_MM_DIFFERENCE = 180.0 / (PI * WHEEL_GAP); // degrees of rotation per unit difference between the left and right wheel travel distances

const float FWD_KP = 1.0;
const float FWD_KD = 1.0;
const float ROT_KP = 4.0;
const float ROT_KD = 0.8;

const int ROBOT_RADIUS = 115; // measure it to the absolute 1mm accuracy
const float RADIANS_PER_DEGREE = PI / 180;

const int PWM_RESOLUTION_BITS = 8;
const int PWM_RESOLUTION = 256;

const int MIN_MOTOR_BIAS = 13;
const int MAX_MOTOR_REACH = 90;        // this is the value the final motor percentages reaches upto
const float MAX_MOTOR_PERCENTAGE = 90; // smaller head room is given for PID
const float MIN_MOTOR_PERCENTAGE = 5;  // when the given percentage is below this value, percentage is set to zero to damp oscillations
// find this value for the all 4 motors and set the maximum from them here

// change motor directions to make the motors spin forward when both motors are forward commanded (for use of incorrect wiring)
#define MOTOR_LEFT_POLARITY (1)
#define MOTOR_RIGHT_POLARITY (-1)
//**********************************************************Sensors**********************************************************

const float STEERING_KP = 1.0;
const float STEERING_KD = 1.0;

const int SENSOR_COUNT = 16;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {4, 5, 6, 7, 15, 16, 17, 18, 8, 3, 9, 10, 11, 12, 13, 14};

const float sensorWeights[SENSOR_COUNT] = {-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5};
//*************************************************************CALIBRATION******************************************


const float THRESHOLD_PERCENTAGE = 0.2;

//*************************************************************SYSTICK*******************************************

#define LOOP_TIME 0.027

//**************************************************************ROBOT*********************************************

const int OMEGA = 540;
const int ALPHA = 3600;

const int RUN_SPEED = 70;
const int RUN_ACC = 1000;

//***********************************************************Reporting***********************************************

#define TX_PIN 2
#define RX_PIN 46

//**********************************************************ADC*******************************************************

#define BATTERY_PIN 19

const float REF_VOLTAGE = 3.3;
const float BAT_ADC_LEVELS = 4095;
const float VOLTAGE_DIVIDER_RATIO = 0.25;

const int MOVING_FILTER_SIZE = 10;

const float NOMINAL_VOLTAGE = 7.4;
const float MIN_VOLTAGE = 7.0;

const int SWITCH_PIN = 7;
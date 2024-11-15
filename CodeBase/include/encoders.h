#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include <math.h>

class Encoders;
extern Encoders encoders;

class Encoders
{
public:
    long tempRight;  // these values will be accessed by printer class for serial print
    long tempLeft; // directly taking values from variables that are getting written by interrupts may crash the program

    void begin()
    {

        pinMode(LeftEncoderPin1, INPUT_PULLUP);
        pinMode(LeftEncoderPin2, INPUT_PULLUP);

        pinMode(RightEncoderPin1, INPUT_PULLUP);
        pinMode(RightEncoderPin2, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(LeftEncoderPin1), updateLeftEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(LeftEncoderPin2), updateLeftEncoderISR, CHANGE);

        attachInterrupt(digitalPinToInterrupt(RightEncoderPin1), updateRightEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RightEncoderPin2), updateRightEncoderISR, CHANGE);

        reset();
    }

    void setLoopTime(float time){
        loopTime = time;
    };

    float getLoopTime(){
        return loopTime;
    };


    void reset()
    {
        noInterrupts();

        encoderCounterLeft = 0;
        encoderCounterRight = 0;

        robot_distance = 0;
        robot_angle = 0;

        interrupts();
    }


    static void updateLeftEncoderISR()
    {
        encoders.updateLeftEncoder();
    }

    static void updateRightEncoderISR()
    {
        encoders.updateRightEncoder();
    }


    void updateLeftEncoder()
    {
        int MSB = digitalRead(LeftEncoderPin1); // Most Significant Bit (A)
        int LSB = digitalRead(LeftEncoderPin2); // Least Significant Bit (B)

        int encoded = (MSB << 1) | LSB; // Create a 2-bit value from A and B

        int sum = (lastEncodedLeft << 2) | encoded; // Combine current and previous states

        // Update position based on the transition
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterLeft--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterLeft++;
        }

        lastEncodedLeft = encoded; // Save the current state
    }

    void updateRightEncoder()
    {
        int MSB = digitalRead(RightEncoderPin1); // Most Significant Bit (A)
        int LSB = digitalRead(RightEncoderPin2); // Least Significant Bit (B)

        int encoded = (MSB << 1) | LSB; // Create a 2-bit value from A and B

        int sum = (lastEncodedRight << 2) | encoded; // Combine current and previous states

        // Update position based on the transition
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterRight--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterRight++;
        }

        lastEncodedRight = encoded; // Save the current state
    }

    void update()
    {
        left_delta = 0;
        right_delta= 0;

        // Make sure values don't change while being read.
        noInterrupts();
        left_delta = encoderCounterLeft;
        right_delta = encoderCounterRight;

        encoderCounterLeft = 0;
        encoderCounterRight = 0;
        interrupts();

        // for serial print
        tempRight = right_delta;
        tempLeft = left_delta;

        float left_change = (float)left_delta * MM_PER_ROTATION_FRONT / PULSES_PER_ROTATION;
        float right_change = (float)right_delta * MM_PER_ROTATION_FRONT / PULSES_PER_ROTATION;


        fwd_change = 0.5 * (right_change + left_change); // taking average, distance in millimeters
        robot_distance += fwd_change;
        rot_change = (right_change - left_change) * DEG_PER_MM_DIFFERENCE;
        robot_angle+= rot_change;
    }





    // front pair values
    inline float robotDistance()
    {
        float distance;

        noInterrupts();
        distance = robot_distance; // in mm
        interrupts();

        return distance;
    }

    inline float robotAngle()
    {
        float angle;

        noInterrupts();
        angle = robot_angle;
        interrupts();

        return angle;
    }

    inline float robot_speed()
    {
        float speed;

        noInterrupts();
        speed = ((float) fwd_change / loopTime);
        interrupts();

        return speed;
    }

    inline float robot_omega()
    { /////given in degrees per second!!!!!
        float omega;

        noInterrupts();
        omega = ((float) rot_change / loopTime);
        interrupts();

        return omega;
    }

    inline float robot_fwd_change()
    {
        float distance;

        noInterrupts();
        distance = fwd_change;
        interrupts();

        return distance;
    }

    inline float robot_rot_change()
    {
        float distance;

        noInterrupts();
        distance = rot_change;
        interrupts();

        return distance;
    }

    inline int leftRPS()
    {
        int rps;

        noInterrupts();
        rps = ((float) left_delta / loopTime);
        interrupts();

        return rps;
    }

    inline int rightRPS()
    {
        int rps;

        noInterrupts();
        rps = ((float) right_delta / loopTime);
        interrupts();

        return rps;
    }

private:
    volatile long encoderCounterLeft; // Encoder roatation count, this gets reset every time we call update
    volatile long lastEncodedLeft;    // Last encoded value

    volatile long encoderCounterRight;
    volatile long lastEncodedRight;

    volatile float robot_distance; // the complete distance travel by robot, this get's incremented using the update function
    volatile float robot_angle;    // same like above

    // the change in distance or angle in the last tick.
    float fwd_change;
    float rot_change;

    int left_delta; // this variable holds the number of encoder counts during two update calls
    int right_delta;

    float loopTime;
};
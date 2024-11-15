#pragma once

#include "Arduino.h"
#include "config.h"
#include "reporting.h"
#include "sensors.h"
#include "systick.h"

class Robot;
extern Robot robot;

class Robot
{
public:
    void run()
    {
        
        while(true){
            LineFeature feature = sensors.detectFeature();

        switch (feature)
        {
        case STRAIGHT_LINE:
            // Normal line following
            sensors.set_steering_mode(STEER_NORMAL);
            followLine();
            break;

        case LEFT_TURN:
            sensors.set_steering_mode(STEERING_OFF);
            handleLeftTurn();
            break;

        case RIGHT_TURN:
            sensors.set_steering_mode(STEERING_OFF);
            handleRightTurn();
            break;

        case T_JUNCTION:
            sensors.set_steering_mode(STEERING_OFF);
            handleTJunction();
            break;

        case CROSS_JUNCTION:
            sensors.set_steering_mode(STEERING_OFF);
            handleCrossJunction();
            break;

        case END_OF_LINE:
            sensors.set_steering_mode(STEERING_OFF);
            handleEndOfLine();
            break;

        case NO_LINE:
            sensors.set_steering_mode(STEERING_OFF);
            handleNoLine();
            break;
        }
        }
        
    }

    void followLine()
    {
        
    }

    // Example motor control function
    void setMotors(int leftSpeed, int rightSpeed)
    {
        // Implement based on your motor driver
        // analogWrite(LEFT_MOTOR_PIN, abs(leftSpeed));
        // analogWrite(RIGHT_MOTOR_PIN, abs(rightSpeed));
        // Set direction pins based on speed sign
    }

    // Implement these based on your competition rules
    void handleLeftTurn()
    {
        // Example: Take sharp left turn
        setMotors(-100, 100);
        delay(100);
    }

    void handleRightTurn()
    {
        // Example: Take sharp right turn
        setMotors(100, -100);
        delay(100);
    }

    void handleTJunction()
    {
        // Example: Default to left turn
        handleLeftTurn();
    }

    void handleCrossJunction()
    {
        // Example: Continue straight
        setMotors(200, 200);
    }

    void handleEndOfLine()
    {
        // Example: Stop and wait
        setMotors(0, 0);
    }

    void handleNoLine()
    {
        // Example: Search for line
        setMotors(-100, 100); // Rotate in place
    }
};
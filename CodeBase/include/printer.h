#pragma once

#include <Arduino.h>
#include "sensors.h"
#include "encoders.h"
#include "motors.h"
// Do not call this function inside the sensors, encoders, motors classes as it will result in circular dependency error
// And call them always after the update functions as it will print the immediate correct values

class Printer;
extern Printer printer;

class Printer
{
public:
    int x = 0;
    int diff = 0;

    void printTimeDiff(bool newLine = false)
    {
        Serial.print(millis() - x);
        if (newLine)
        {
            Serial.print("\n");
        }
        else
        {
            Serial.print("  ");
        }
        x = millis();
    }

    
    void printEncoderCounts(bool newline = false)
    {
        Serial.print("Encoder Counts : Right: ");
        Serial.print(encoders.tempRight);
        Serial.print("| Left: ");
        Serial.print(encoders.tempLeft);

        if (newline)
        {
            Serial.print("\n");
        }
        else
        {
            Serial.print("  ");
        }
    }

    void printMotorFeedPercentages(bool newline = false)
    {
        Serial.print("Motor Percentages || Right: ");
        Serial.print(motors.right_motor_percentage);
        Serial.print("| Left: ");
        Serial.print(motors.left_motor_percentage);

        if (newline)
        {
            Serial.print("\n");
        }
        else
        {
            Serial.print("  ");
        }
    }

    void printSteeringAdjustment(bool newline)
    {
        Serial.print("Seteering adjust : ");
        Serial.print(sensors.get_steering_adjustment());

        if (newline)
        {
            Serial.print("\n");
        }
        else
        {
            Serial.print("  ");
        }
    }
};
#pragma once

#include <Arduino.h>
#include "sensors.h"
#include "motors.h"
#include "encoders.h"
#include "config.h"

class Calibaration;
extern Calibaration calibaration;

class Calibaration
{
private:
    // variables for motor feed forward calibaration
    const int SAMPLES_PER_PERCENTAGE = 50; // Number of samples to average for each percentage
    const int STABILIZE_DELAY = 500;       // Time in ms to wait for motor speed to stabilize
    const float PERCENTAGE_STEP = 1.0;     // Step size for percentage increments

public:
    struct MotorData
    {
        float percentage;
        float rps_left;
        float rps_right;
    };

    // Motor feed forward calibration functions
    //  Function to collect data for a single percentage value
    MotorData measureAtPercentage(float percentage)
    {
        MotorData data;
        data.percentage = percentage;

        // Set all motors to the same percentage
        motors.set_left_motor_percentage(percentage);
        motors.set_right_motor_percentage(percentage);

        // Wait for speed to stabilize
        delay(STABILIZE_DELAY);

        // Take multiple samples and average them
        float l_sum = 0, r_sum = 0;

        for (int i = 0; i < SAMPLES_PER_PERCENTAGE; i++)
        {
            l_sum += abs((float)encoders.leftRPS());
            r_sum += abs((float)encoders.rightRPS());
            delay(20); // Small delay between samples
        }

        data.rps_left = l_sum / SAMPLES_PER_PERCENTAGE;
        data.rps_right = r_sum / SAMPLES_PER_PERCENTAGE;

        return data;
    }

    // Function to run a complete calibration sequence
    void runMotorCalibration()
    {
        Serial.println("Starting motor calibration...");
        Serial.println("Percentage, LB_RPS, RB_RPS, LF_RPS, RF_RPS");

        // Test from MIN_MOTOR_PERCENTAGE to MAX_MOTOR_PERCENTAGE
        for (float percentage = MIN_MOTOR_PERCENTAGE; percentage <= MAX_MOTOR_PERCENTAGE; percentage += PERCENTAGE_STEP)
        {

            MotorData data = measureAtPercentage(percentage);
            printMotorData(data);
            delay(30); // Brief pause between measurements
        }

        delay(30);
        motors.coast();
        delay(500);
        // Test negative percentages
        for (float percentage = -MIN_MOTOR_PERCENTAGE; percentage >= -MAX_MOTOR_PERCENTAGE; percentage -= PERCENTAGE_STEP)
        {

            MotorData data = measureAtPercentage(percentage);
            printMotorData(data);
            delay(30);
        }
        delay(30);
        motors.coast();
        delay(500);

        // Stop motors after calibration
        motors.stop();
        Serial.println("Calibration complete!");
    }

    // Don't need to move this function to the printer class, as this will be only used one time for the entire project
    // unless current limited power sources and motors didn't change
    void printMotorData(const MotorData &data)
    {
        char buffer[100];
        snprintf(buffer, sizeof(buffer),
                 "%6.2f, %6.2f, %6.2f, %6.2f, %6.2f",
                 data.percentage,
                 data.rps_left,
                 data.rps_right);
        Serial.println(buffer);
    }

    // Auto-calibration function, this will be called in the sensors.begin()
    void sensorAutoCalibrate(int duration_ms = 4000)
    {
        unsigned long startTime = millis();

        // Initialising the arrays or Clearing the previous calibration
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            sensors.minValues[i] = 4095;
            sensors.maxValues[i] = 0;
        }

        // Collecting samples for specified duration
        while (millis() - startTime < duration_ms)
        {
            for (int i = 0; i < SENSOR_COUNT; i++)
            {
                uint16_t value = analogRead(SENSOR_PINS[i]);
                // chosing the minimum and maximum value, get read for each sensor
                sensors.minValues[i] = min(sensors.minValues[i], value);
                sensors.maxValues[i] = max(sensors.maxValues[i], value);
            }
            delay(2);
        }

        // Calculate the black, white differentiating thresholds
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            // Setting threshold above minimum by some percentage to avoid noise
            uint16_t threshold = sensors.minValues[i] + (sensors.maxValues[i] - sensors.minValues[i]) * THRESHOLD_PERCENTAGE;
            sensors.thresholds[i] = map(threshold, sensors.minValues[i], sensors.maxValues[i], 0 , 1000 );
        }
    }
};
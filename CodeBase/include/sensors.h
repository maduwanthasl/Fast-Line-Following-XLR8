#pragma once

#include <Wire.h>
#include "config.h"
#include "encoders.h"

class Sensors;
extern Sensors sensors;

enum
{
    STEER_NORMAL,
    STEERING_OFF,
};

enum
{
    FAST_MODE,
    SLOW_MODE
};

// Detect special features
enum LineFeature
{
    STRAIGHT_LINE,
    LEFT_TURN,
    RIGHT_TURN,
    T_JUNCTION,
    CROSS_JUNCTION,
    END_OF_LINE,
    NO_LINE
};

class Sensors
{
private:
    float last_line_error = 0;
    volatile float line_error;
    volatile float steering_adjustment;
    bool onLine = false;

    uint16_t line_pattern = 0;

public:
    float steering_kp = STEERING_KP;
    float steering_kd = STEERING_KD;

    bool enableIR = false;

    uint8_t steering_mode = STEER_NORMAL;

    // Calibration values
    uint16_t minValues[SENSOR_COUNT];
    uint16_t maxValues[SENSOR_COUNT];
    uint16_t thresholds[SENSOR_COUNT];

    uint16_t rawValues[SENSOR_COUNT];
    uint16_t normalizedValues[SENSOR_COUNT];

    // Initialize all sensors
    void begin()
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            pinMode(SENSOR_PINS[i], INPUT);
        }
    }

    // to not take readings and save time
    void enableIR_Readings()
    {
        enableIR = true;
    }

    void disableIR_Readings()
    {
        enableIR = false;
    }

    float get_steering_adjustment()
    {
        return steering_adjustment;
    }

    void set_steering_mode(uint8_t mode)
    {
        last_line_error = line_error;
        steering_adjustment = 0;
        steering_mode = mode;
    }

    void calculate_steering_adjustment()
    {
        float pTerm = steering_kp * line_error;
        float dTerm = steering_kd * (line_error - last_line_error) / LOOP_TIME;

        float adjustment = (pTerm + dTerm) * LOOP_TIME;

        if (onLine)
            last_line_error = line_error; // only update the last_line_error if the line hasn't completely went out of the sight

        steering_adjustment = adjustment;
    }

    void update()
    {
        readLine(true);
        calculate_steering_adjustment();
        detectLinePattern();
    }

    float readLine(bool isBlackLine)
    {
        bool lineFound = false;
        line_error = 0;

        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            rawValues[i] = analogRead(SENSOR_PINS[i]);

            // map the sensor read into value between 0 and 10000
            normalizedValues[i] = map(rawValues[i], minValues[i], maxValues[i], 0, 1000);

            if (isBlackLine)
            {
                if (normalizedValues[i] > thresholds[i])
                {
                    line_error += i * sensorWeights[i];
                    lineFound = true;
                }
            }
            else
            {
            }
        }

        onLine = lineFound;

        if (!lineFound)
        {
            // if line lost, use last known error
            return last_line_error;
        }

        return line_error;
    }

    // Get binary line pattern for advanced detection
    void detectLinePattern()
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (normalizedValues[i] > thresholds[i])
            {
                line_pattern |= (1 << i); // it would be easy to debug if pins were assigned in such a way that first index is the left most sensor
            }
        }
    }

    LineFeature detectFeature()
    {
        detectLinePattern();

        if (line_pattern == 0)
            return NO_LINE;

        // Count active sensors
        int activeSensors = 0;
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (line_pattern & (1 << i))
                activeSensors++;
        }

        // Analyze pattern
        if (activeSensors > 12)
            return CROSS_JUNCTION;

        if (activeSensors > 8)
            return T_JUNCTION;

        if ((line_pattern & 0xFF00) && activeSensors > 3)
            return LEFT_TURN;

        if ((line_pattern & 0x00FF) && activeSensors > 3)
            return RIGHT_TURN;

        if (activeSensors < 2)
            return END_OF_LINE;
        
        return STRAIGHT_LINE;
    }

    bool isOnLine()
    {
        return onLine;
    }

    float getLastLineError()
    {
        return last_line_error;
    }

    uint16_t getRawValue(int sensor)
    {
        return rawValues[sensor];
    }

    uint16_t getNormalizedValue(int sensor)
    {
        return normalizedValues[sensor];
    }

    uint16_t getLinePattern(){
        return line_pattern;
    }
};
#pragma once

#include <Arduino.h>
#include <config.h>

class ADC;

extern ADC adc;

class ADC
{
private:
    bool simulationMode = false;

public:
    void enableSimulation(bool enable)
    {
        simulationMode = enable;
        if (enable)
        {
            Serial.println("Switch simulation mode enabled. Enter values 0-6 to simulate switch states.");
        }
    }

    int realSwitchRead()
    {
        int analogValue = analogRead(SWITCH_PIN);
        if (analogValue > 3412)
            return 1;
        else if (analogValue > 2730)
            return 2;
        else if (analogValue > 2048)
            return 3;
        else if (analogValue > 1365)
            return 4;
        else if (analogValue > 683)
            return 5;
        else if (analogValue > 10)
            return 6;
        return 0;
    }

    int simulatedSwitchRead()
    {
        while (true)
        { // This creates a blocking read
            if (Serial.available() > 0)
            {
                int input = Serial.parseInt();
                Serial.read(); // Clear the buffer
                if (input >= 0 && input <= 6)
                {
                    Serial.print("Simulated switch value: ");
                    Serial.println(input);
                    return input;
                }
                else
                {
                    Serial.println("Invalid input. Please enter a value between 0 and 6.");
                }
            }
            delay(10);
        }
    }

    int switchRead()
    {
        if (simulationMode)
        {
            return simulatedSwitchRead();
        }
        else
        {
            return realSwitchRead();
        }
    }


    void updateBattery() {
        // Read raw value and convert to voltage
        int raw_value = analogRead(BATTERY_PIN);
        float measured_voltage = (raw_value * REF_VOLTAGE / BAT_ADC_LEVELS) / VOLTAGE_DIVIDER_RATIO;
        
        // Update moving average
        voltage_readings[reading_index] = measured_voltage;
        reading_index = (reading_index + 1) % MOVING_FILTER_SIZE;
        
        // Calculate average
        float sum = 0;
        for(int i = 0; i < MOVING_FILTER_SIZE; i++) {
            sum += voltage_readings[i];
        }
        current_voltage = sum / MOVING_FILTER_SIZE;
    }

    float getCurrentVoltage() {
        return current_voltage;
    }

    bool isLowBattery() {
        return current_voltage < MIN_VOLTAGE;
    }

    inline float getCompensationFactor(){
        compensation_factor = NOMINAL_VOLTAGE / current_voltage;

        return compensation_factor;
    }

private:
    float current_voltage = NOMINAL_VOLTAGE;
    float voltage_readings[MOVING_FILTER_SIZE];
    int reading_index = 0;
    float compensation_factor;
};
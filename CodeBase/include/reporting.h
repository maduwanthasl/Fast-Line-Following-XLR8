#ifndef REPORTING_H
#define REPORTING_H

#include "motors.h"
#include "sensors.h"
#include <config.h>

class Reporting;
extern Reporting reporter;

class Reporting
{
public:
    int speed, omega;

    typedef struct sendData
    {
        float speedData;
        float omegaData;
        float distance;
        float angle;
    } sendData;

    typedef struct receiveData
    {
        float fwdKp;
        float fwdKd;
        float rotKp;
        float rotKd;
        float steeringKp;
        float steeringKd;
        int speed;
        int omega;
    } receiveData;

    receiveData command;
    sendData transmitData;

    void checkReceived()
    {
        if (Serial2.available())
        {
            String is_ACK = Serial2.readStringUntil('\n');
            if (is_ACK == "SEND_OK" || is_ACK == "SEND_FAIL")
            {
                Serial.println(is_ACK);
            }
            else
            {
                motors.fwdKp = is_ACK.toFloat();
                motors.fwdKd = Serial2.parseFloat();
                motors.rotKp = Serial2.parseFloat();
                motors.rotKd = Serial2.parseFloat();
                sensors.steering_kp = Serial2.parseFloat();
                sensors.steering_kd = Serial2.parseFloat();

                speed = Serial2.parseInt();
                omega = Serial2.parseInt();

                Serial.printf("fwdKp: %f, fwdKd: %f, rotKp: %f, rotKd: %f, steeringKp: %f, steeringKd: %f, speed: %d, omega: %d \n", motors.fwdKp, motors.fwdKd, motors.rotKp, motors.rotKd, sensors.steering_kp, sensors.steering_kd, speed, omega);
            }
        }
    }

    void begin()
    {
        Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    }

    void sendDebugData()
    {
        Serial2.printf("%f %f %f %f", encoders.robot_speed(), encoders.robot_omega(), encoders.robotDistance(), encoders.robotAngle());
    }
};

#endif
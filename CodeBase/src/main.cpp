#include <Arduino.h>
#include "systick.h"
#include "config.h"
#include "adc.h"
#include "sensors.h"
#include "printer.h"
#include "calibaration.h"
#include "motion.h"
#include "profile.h"
#include "robot.h"

Motors motors;
Encoders encoders;
Sensors sensors;
Systick systick;
ADC adc;
Printer printer;
Calibaration calibaration;
Robot robot;
Motion motion;
Profile rotation;
Profile forward;

void setup()
{
  Serial.begin(115200);

  encoders.begin();
  motors.begin();
  sensors.begin();
  systick.begin();

  motion.reset_drive_system();
  calibaration.sensorAutoCalibrate();

  sensors.set_steering_mode(STEERING_OFF);

  // Optional: Use motors to move during calibration
  // setMotors(50, -50);  // Rotate in place
  calibaration.sensorAutoCalibrate();
  // setMotors(0, 0);
}

void loop()
{
  robot.run();
}
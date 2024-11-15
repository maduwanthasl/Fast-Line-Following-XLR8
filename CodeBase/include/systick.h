#pragma once

#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "Ticker.h"
#include "printer.h"
#include "motion.h"
#include "config.h"

class Systick;
extern Systick systick;

class Systick
{
private:
    double tickerTime = LOOP_TIME;
    bool slowMode = false;
    Ticker ticker;

public:
    bool isSlowModeEnabled()
    {
        return slowMode;
    }

    void begin()
    {
        encoders.setLoopTime(tickerTime);
        ticker.attach(tickerTime, []()
                      {
                          encoders.update();
                          sensors.update();
                          motors.update(motion.velocity(), motion.omega(), sensors.get_steering_adjustment());

                        //   printer.printTimeDiff();
                        //   printer.printEncoderCounts();
                        //   printer.printSteeringAdjustment(false);
                        //   printer.printMotorFeedPercentages(true);
                      });
    }
};

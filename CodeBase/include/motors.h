#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "adc.h"

class Motors;
extern Motors motors;

class Motors
{
private:
  float previous_fwd_error;
  float previous_rot_error;
  float fwd_error;
  float rot_error;

  float velocity;
  float omega;

  bool feedforward_enabled = true;
  bool controller_output_enabled;

public:
  // remove this after testing

  float fwdKp = FWD_KP;
  float fwdKd = FWD_KD;
  float rotKp = ROT_KP;
  float rotKd = ROT_KD;

  float left_motor_percentage;
  float right_motor_percentage;

  void begin()
  {
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);

    digitalWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR_IN1, 0);
    digitalWrite(LEFT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR_IN1, 0);
    digitalWrite(RIGHT_MOTOR_IN2, 0);

    setupPWM();
  }

  void setupPWM()
  {
    ledcSetup(0, 5000, PWM_RESOLUTION_BITS); // check for different pwm frequencies
    ledcAttachPin(LEFT_MOTOR_PWM, 0);
    ledcSetup(1, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_MOTOR_PWM, 1);
  }

  void reset_controllers()
  {
    fwd_error = 0;
    rot_error = 0;
    previous_fwd_error = 0;
    previous_rot_error = 0;
  }

  void enable_controllers()
  {
    controller_output_enabled = true;
  }

  void disable_controllers()
  {
    controller_output_enabled = false;
  }

  void stop()
  {
    set_left_motor_percentage(0);
    set_right_motor_percentage(0);
  }

  float position_controller()
  {
    float increment = velocity * encoders.getLoopTime();
    fwd_error += increment - encoders.robot_fwd_change();
    float diff = fwd_error - previous_fwd_error;
    previous_fwd_error = fwd_error;

    // change them to config kp kd later
    float output = fwdKp * fwd_error + fwdKd * diff;
    return output;
  }

  float angle_controller(float steering_adjustment)
  {
    float increment = omega * encoders.getLoopTime();
    rot_error += increment - encoders.robot_rot_change();
    float diff = rot_error - previous_rot_error;
    previous_rot_error = rot_error;

    rot_error -= steering_adjustment;

    // changethis kp kd to config kp kd later
    float output = rotKp * rot_error + rotKd * diff;
    return output;
  }

  // feed forward functions gives the percentage required to acheive a given velocity
  float left_feed_forward_percentage(float left_front_feed_velocity)
  {
    int l_rps = (left_front_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION_FRONT;
    float l_feed_percentage;
    if (l_rps >= 0)
    {
      l_feed_percentage = 0.0220 * l_rps - 3.4823;
    }
    else
    {
      l_feed_percentage = 0.0217 * l_rps + 5.2456;
    }

    return l_feed_percentage;
  }

  float right_feed_forward_percentage(float right_front_feed_velocity)
  {
    int r_rps = (right_front_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION_FRONT;

    float r_feed_percentage;
    if (r_rps >= 0)
    {
      r_feed_percentage = 0.0215 * r_rps - 2.6277;
    }
    else
    {
      r_feed_percentage = 0.0225 * r_rps + 5.3887;
    }

    return r_feed_percentage;
  }



  void update(float vel, float omg, float steering)
  {
    velocity = vel;
    omega = omg;

    // common parameters for both front and back pairs
    float tangent_speed = omega * ROBOT_RADIUS * RADIANS_PER_DEGREE;
    float left_speed = velocity - tangent_speed;
    float right_speed = velocity + tangent_speed;

    // pair-wise calculation for two pairs of motors
    float pos_output = position_controller();
    float rot_output = angle_controller(steering);

    float left_output = 0;
    float right_output = 0;

    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;

    float left_ff = left_feed_forward_percentage(left_speed);
    float right_ff = right_feed_forward_percentage(right_speed);

    if (feedforward_enabled)
    {
      left_output += left_ff;
      right_output += right_ff;
    }

    if (controller_output_enabled)
    {
      set_left_motor_percentage(left_output);
      set_right_motor_percentage(right_output);
    }
  }

  void set_left_motor_percentage(float percentage)
  {
    percentage = constrainPercentage(percentage);

    left_motor_percentage = percentage;
    int left_pwm = calculate_pwm(percentage);
    set_left_motor_pwm(left_pwm);
  }

  void set_right_motor_percentage(float percentage)
  {
    percentage = constrainPercentage(percentage);

    right_motor_percentage = percentage;
    int right_pwm = calculate_pwm(percentage);
    set_right_motor_pwm(right_pwm);
  }

  // limits the given percentage to minimum bias and maximum value (needs to overcome motor inequalities at lower pwm,
  // friction and give headroom for PID values to adjust in the higher edge)
  float constrainPercentage(float percentage)
  {
    percentage = constrain(percentage, -MAX_MOTOR_PERCENTAGE, MAX_MOTOR_PERCENTAGE);
    if (percentage > MIN_MOTOR_PERCENTAGE)
    {
      percentage = map(percentage, MIN_MOTOR_PERCENTAGE, MAX_MOTOR_PERCENTAGE, MIN_MOTOR_BIAS, MAX_MOTOR_REACH);
    }
    else if (percentage < -MIN_MOTOR_PERCENTAGE)
    {
      percentage = map(percentage, -MAX_MOTOR_PERCENTAGE, -MIN_MOTOR_PERCENTAGE, -MAX_MOTOR_REACH, -MIN_MOTOR_BIAS);
    }
    else if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
    {
      percentage = 0;
    }

    return percentage;
  }

  // sets pwm values and directions by these seperate functions for each motor
  void set_left_motor_pwm(int pwm)
  {
    pwm = MOTOR_LEFT_POLARITY * pwm;
    if (pwm < 0)
    {
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      ledcWrite(0, -pwm);
    }
    else
    {
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      ledcWrite(0, pwm);
    }
  }

  void set_right_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_POLARITY * pwm;
    if (pwm < 0)
    {
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      ledcWrite(1, -pwm);
    }
    else
    {
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
      ledcWrite(1, pwm);
    }
  }


  int calculate_pwm(float desired_percentage)
  {
    int pwm = MAX_MOTOR_PERCENTAGE * PWM_RESOLUTION * desired_percentage / 10000;
    pwm = batteryCompPWM(pwm);
    return pwm;
  }

  int batteryCompPWM(int pwm) {
    int adjustedPWM = pwm * adc.getCompensationFactor();
    if (adjustedPWM>PWM_RESOLUTION){
      adjustedPWM = PWM_RESOLUTION;
    }
    return adjustedPWM;
}

  // mainly for using in the calibration class
  void coast()
  {
    // Set the control pins for each motor to LOW, allowing them to coast
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);

    // Set PWM to 0 for all motors to ensure no active power is applied
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
};
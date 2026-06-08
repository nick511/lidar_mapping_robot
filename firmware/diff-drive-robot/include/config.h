#ifndef CONFIG_H
#define CONFIG_H

namespace Pins
{
  // Motor Standby
  constexpr int MOTOR_STBY = 19;

  // Left Motor
  constexpr int L_PWM = 25;
  constexpr int L_IN1 = 26;
  constexpr int L_IN2 = 27;

  // Right Motor
  constexpr int R_PWM = 14;
  constexpr int R_IN1 = 12;
  constexpr int R_IN2 = 13;

  // Encoders
  constexpr int L_ENC_A = 34;
  constexpr int L_ENC_B = 35;
  constexpr int R_ENC_A = 32;
  constexpr int R_ENC_B = 33;
}

namespace RobotConfig
{
  constexpr float WHEEL_BASE = 0.26;     // meters
  constexpr float WHEEL_DIAMETER = 0.07; // meters
  constexpr float TICKS_PER_REV_L = 492;
  constexpr float TICKS_PER_REV_R = 493;
}

#endif

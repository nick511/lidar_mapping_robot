#include "motor_helpers.h"
#include "config.h"

#include <Arduino.h>

// ======================================================
// PWM CONFIG
// ======================================================
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define L_PWM_CHANNEL 0
#define R_PWM_CHANNEL 1

// ======================================================
// MOTOR SETUP
// ======================================================
void motorSetup()
{
  pinMode(Pins::L_IN1, OUTPUT);
  pinMode(Pins::L_IN2, OUTPUT);
  pinMode(Pins::R_IN1, OUTPUT);
  pinMode(Pins::R_IN2, OUTPUT);
  pinMode(Pins::MOTOR_STBY, OUTPUT);

  // ==========================================
  // PWM SETUP
  // ==========================================
  ledcSetup(L_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Pins::L_PWM, L_PWM_CHANNEL);
  ledcSetup(R_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Pins::R_PWM, R_PWM_CHANNEL);
}

// ======================================================
// ENCODER ISR
// ======================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;
void IRAM_ATTR leftEncoderISR()
{
  if (digitalRead(Pins::L_ENC_B))
    left_ticks++;
  else
    left_ticks--;
}

void IRAM_ATTR rightEncoderISR()
{
  if (!digitalRead(Pins::R_ENC_B))
    right_ticks++;
  else
    right_ticks--;
}

// ======================================================
// ENCODER SETUP
// ======================================================
void encoderSetup()
{
  pinMode(Pins::L_ENC_A, INPUT_PULLUP);
  pinMode(Pins::L_ENC_B, INPUT_PULLUP);
  pinMode(Pins::R_ENC_A, INPUT_PULLUP);
  pinMode(Pins::R_ENC_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(Pins::L_ENC_A),
      leftEncoderISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(Pins::R_ENC_A),
      rightEncoderISR,
      RISING);
}

void enableMotors()
{
  digitalWrite(Pins::MOTOR_STBY, HIGH);
}

void disableMotors()
{
  digitalWrite(Pins::MOTOR_STBY, LOW);
}

// ======================================================
// MOTOR CONTROL
// ======================================================
void setMotor(
    int pwmChannel,
    int in1,
    int in2,
    float speed)
{
  speed = constrain(speed, -1.0, 1.0);

  int pwm = abs(speed) * 255;

  if (speed > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (speed < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmChannel, pwm);
}

// ==================================================
// SPEED CONTROL
// ==================================================
const float KP = 0.5;
float getPwm(float target_rpm, float actual_rpm, float pwm)
{
  // ==================================================
  // P CONTROLLER
  // ==================================================
  float error = target_rpm - actual_rpm;
  pwm += KP * error;

  // Clamp PWM
  return constrain(pwm, -255, 255);
}

float ticksToRPM(long delta_ticks, float delta_time)
{
  float revs = (float)delta_ticks / RobotConfig::TICKS_PER_REV;

  return (revs / delta_time) * 60.0;
}

// ======================================================
// MOTOR CONTROL BY VELOCITY
// ======================================================
long prev_left_ticks = 0;
long prev_right_ticks = 0;
float left_pwm = 0;
float right_pwm = 0;
float wheel_circumference = PI * RobotConfig::WHEEL_DIAMETER;

void setMotorByVel(float linear_x, float angular_z, float delta_time)
{
  // ==================================================
  // TARGET WHEEL VELOCITIES
  // ==================================================
  float left_linear =
      linear_x - (angular_z * RobotConfig::WHEEL_BASE / 2.0);

  float right_linear =
      linear_x + (angular_z * RobotConfig::WHEEL_BASE / 2.0);

  // ==================================================
  // CONVERT TO TARGET RPM
  // ==================================================
  float left_target_rpm =
      (left_linear / wheel_circumference) * 60.0;

  float right_target_rpm =
      (right_linear / wheel_circumference) * 60.0;

  // ======================================
  // READ ENCODERS
  // ======================================
  long l_ticks = left_ticks;
  long r_ticks = right_ticks;

  long dl = l_ticks - prev_left_ticks;
  long dr = r_ticks - prev_right_ticks;

  prev_left_ticks = l_ticks;
  prev_right_ticks = r_ticks;

  // ==================================================
  // ACTUAL RPM
  // ==================================================
  float left_actual_rpm = ticksToRPM(dl, delta_time);
  float right_actual_rpm = ticksToRPM(dr, delta_time);

  left_pwm = getPwm(left_target_rpm, left_actual_rpm, left_pwm);
  right_pwm = getPwm(right_target_rpm, right_actual_rpm, right_pwm);

  // ======================================
  // MOTOR OUTPUT
  // ======================================
  setMotor(
      L_PWM_CHANNEL,
      Pins::L_IN1,
      Pins::L_IN2,
      left_pwm / 255.0);

  setMotor(
      R_PWM_CHANNEL,
      Pins::R_IN1,
      Pins::R_IN2,
      right_pwm / 255.0);

  // ==================================================
  // DEBUG
  // ==================================================
  Serial.print("L target: ");
  Serial.print(left_target_rpm);

  Serial.print("  L actual: ");
  Serial.print(left_actual_rpm);

  Serial.print("  PWM: ");
  Serial.print(left_pwm);

  Serial.print(" || R target: ");
  Serial.print(right_target_rpm);

  Serial.print("  R actual: ");
  Serial.print(right_actual_rpm);

  Serial.print("  PWM: ");
  Serial.println(right_pwm);
}

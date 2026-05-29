#include "motor_helpers.h"

#include <Arduino.h>

// ======================================================
// MOTOR PINS
// ======================================================
#define L_PWM 25
#define L_IN1 26
#define L_IN2 27
#define R_PWM 14
#define R_IN1 12
#define R_IN2 13
#define MOTOR_STBY 19

// ======================================================
// ENCODER PINS
// ======================================================
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33

// ======================================================
// PWM CONFIG
// ======================================================
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define L_PWM_CHANNEL 0
#define R_PWM_CHANNEL 1

// ======================================================
// ENCODER COUNTERS
// ======================================================
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// ======================================================
// ROBOT PARAMETERS
// ======================================================
const float wheel_base = 0.14; // meters
const float max_speed = 0.5;   // m/s

// ======================================================
// ENCODER ISR
// ======================================================
void IRAM_ATTR leftEncoderISR()
{
  if (digitalRead(L_ENC_B))
    left_ticks++;
  else
    left_ticks--;
}

void IRAM_ATTR rightEncoderISR()
{
  if (digitalRead(R_ENC_B))
    right_ticks++;
  else
    right_ticks--;
}

void motorSetup()
{
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // ==========================================
  // PWM SETUP
  // ==========================================
  ledcSetup(L_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_PWM, L_PWM_CHANNEL);
  ledcSetup(R_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(R_PWM, R_PWM_CHANNEL);
}

void encoderSetup()
{
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(L_ENC_A),
      leftEncoderISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(R_ENC_A),
      rightEncoderISR,
      RISING);
}

void enableMotors()
{
  digitalWrite(MOTOR_STBY, HIGH);
}

void disableMotors()
{
  digitalWrite(MOTOR_STBY, LOW);
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

long prev_left_ticks = 0;
long prev_right_ticks = 0;

void setMotorByVel(float linear_x, float angular_z)
{
  // ======================================
  // DIFFERENTIAL DRIVE KINEMATICS
  // ======================================

  float left_speed =
      linear_x - (angular_z * wheel_base / 2.0);

  float right_speed =
      linear_x + (angular_z * wheel_base / 2.0);

  // Normalize to [-1, 1]
  left_speed /= max_speed;
  right_speed /= max_speed;

  // ======================================
  // MOTOR OUTPUT
  // ======================================

  setMotor(
      L_PWM_CHANNEL,
      L_IN1,
      L_IN2,
      left_speed);

  setMotor(
      R_PWM_CHANNEL,
      R_IN1,
      R_IN2,
      right_speed);

  // ======================================
  // RPM DEBUG
  // ======================================
  long l_ticks = left_ticks;
  long r_ticks = right_ticks;

  long dl = l_ticks - prev_left_ticks;
  long dr = r_ticks - prev_right_ticks;

  prev_left_ticks = l_ticks;
  prev_right_ticks = r_ticks;

  return;

  Serial.print("L ticks: ");
  Serial.print(dl);

  Serial.print("  R ticks: ");
  Serial.println(dr);
}

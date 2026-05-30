#include "ros_helpers.h"
#include "motor_helpers.h"

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <geometry_msgs/msg/twist.h>

// ======================================================
// CMD_VEL CALLBACK
// ======================================================
float linear_x = 0.0;
float angular_z = 0.0;
void cmdVelCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *twist =
      (const geometry_msgs__msg__Twist *)msgin;

  linear_x = twist->linear.x;
  angular_z = twist->angular.z;
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Serial.begin(115200);

  motorSetup();
  encoderSetup();
  enableMotors();

  // ==========================================
  // micro-ROS & ROS INIT
  // ==========================================
  micro_ros_setup((char *)WIFI_SSID, (char *)WIFI_PASS, (char *)AGENT_IP, AGENT_PORT, "diff_bot_esp32");
  ros_subscription_init("cmd_vel", cmdVelCallback);
}

// ======================================================
// MAIN LOOP
// ======================================================
unsigned long last_control = 0;

void loop()
{
  ros_subscription();

  // ==========================================
  // 50Hz CONTROL LOOP
  // ==========================================
  if (millis() - last_control >= 20)
  {
    unsigned long now = millis();
    float dt = (now - last_control) / 1000.0;
    last_control = now;

    setMotorByVel(linear_x, angular_z, dt);
  }
}

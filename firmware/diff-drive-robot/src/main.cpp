#include "wifi_helpers.h"
#include "ros_helpers.h"
#include "motor_helpers.h"

#include <Arduino.h>
#include <ArduinoOTA.h>

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

  wifi_setup(WIFI_SSID, WIFI_PASS);
  ota_setup();

  // ==========================================
  // MOTOR SETUP
  // ==========================================
  motorSetup();
  encoderSetup();
  enableMotors();

  // ==========================================
  // micro-ROS SETUP
  // ==========================================
  micro_ros_setup((char *)WIFI_SSID, (char *)WIFI_PASS, (char *)AGENT_IP, AGENT_PORT, "diff_bot_esp32");
  ros_subscription_init("cmd_vel", cmdVelCallback);
  ros_publisher_init("odom");
}

// ======================================================
// MAIN LOOP
// ======================================================
unsigned long last_control_50hz = 0;
unsigned long last_control_20hz = 0;

void loop()
{
  ota_handle();

  ros_subscription();

  // ==========================================
  // 50Hz CONTROL LOOP
  // ==========================================
  if (millis() - last_control_50hz >= 20)
  {
    unsigned long now = millis();
    float dt = (now - last_control_50hz) / 1000.0;
    last_control_50hz = now;

    setMotorByVel(linear_x, angular_z, dt);
  }

  // ==========================================
  // 20Hz CONTROL LOOP
  // ==========================================
  if (millis() - last_control_20hz >= 50)
  {
    unsigned long now = millis();
    float dt = (now - last_control_20hz) / 1000.0;
    last_control_20hz = now;

    ros_publishOdometry(updateOdometry(dt));
  }
}

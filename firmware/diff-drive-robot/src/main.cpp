#include "wifi_helpers.h"
#include "ros_helpers.h"
#include "motor_helpers.h"
#include "sensor_helpers.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ICM_20948.h>

#include <micro_ros_platformio.h>
#include <geometry_msgs/msg/twist.h>

const int READY_LED_PIN = 2;
const bool IMU_ENABLED = true;

ICM_20948_I2C imu;

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

  // Initialize READY LED
  pinMode(READY_LED_PIN, OUTPUT);

  // Wifi and OTA
  wifi_setup(WIFI_SSID, WIFI_PASS);
  ota_setup();

  // ==========================================
  // MOTOR SETUP
  // ==========================================
  motorSetup();
  encoderSetup();
  enableMotors();

  // ==========================================
  // SENSOR SETUP
  // ==========================================
  if (IMU_ENABLED)
  {
    Wire.begin();
    Wire.setClock(400000);
    imuSetup(&imu);
  }

  // ==========================================
  // micro-ROS SETUP
  // ==========================================
  micro_ros_setup((char *)WIFI_SSID, (char *)WIFI_PASS, (char *)AGENT_IP, AGENT_PORT, "diff_bot_esp32");
  ros_subscription_init("cmd_vel", cmdVelCallback);
  ros_publisher_init();

  // Indicate setup is complete
  digitalWrite(READY_LED_PIN, HIGH);
}

// ======================================================
// MAIN LOOP
// ======================================================
unsigned long last_control_50hz = 0;
unsigned long last_control_20hz = 0;

void loop()
{
  ota_handle();

  ros_sync_clock();
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

    // IMU
    float gyro_z_clean = 0.0f;
    if (IMU_ENABLED)
    {
      imu.getAGMT();
      gyro_z_clean = getCleanGyroZ(&imu); // we may not need this after EKF
      ros_publishIMU(imu.accX(), imu.accY(), imu.accZ(), gyro_z_clean);
    }

    // ODOMETRY
    ros_publishOdometry(updateOdometry(dt, IMU_ENABLED, gyro_z_clean * DEG_TO_RAD));
  }
}

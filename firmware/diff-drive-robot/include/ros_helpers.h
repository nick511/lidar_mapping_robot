#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

#include "types.h"

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

void micro_ros_setup(char *ssid, char *pass, char *agent_ip, uint16_t agent_port, const char *node_name);
void ros_publisher_init();
void ros_subscription_init(const char *topic_name, rclc_subscription_callback_t callback);

void ros_subscription();
void ros_publishOdometry(Pose pose);
void ros_publishIMU(float acc_x, float acc_y, float acc_z, float gyr_z);

#endif

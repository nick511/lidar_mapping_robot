#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

#include "types.h"

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

void micro_ros_setup(char *ssid, char *pass, char *agent_ip, uint16_t agent_port, const char *node_name);
void ros_publisher_init(const char *topic_name);
void init_odom_msg();
void ros_subscription_init(const char *topic_name, rclc_subscription_callback_t callback);

void ros_subscription();
void ros_publish(const nav_msgs__msg__Odometry &msg);
void ros_publishOdometry(Pose pose);

#endif

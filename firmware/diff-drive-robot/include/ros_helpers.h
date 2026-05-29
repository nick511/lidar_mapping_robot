#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <led_control_msgs/msg/led_status.h>

void micro_ros_setup(char *ssid, char *pass, char *agent_ip, uint16_t agent_port, const char *node_name);
void ros_publisher_init(const char *topic_name);
void ros_subscription_init(const char *topic_name, rclc_subscription_callback_t callback);

// void ros_publish(led_control_msgs__msg__LedStatus led_msg);
void ros_subscription();

#endif

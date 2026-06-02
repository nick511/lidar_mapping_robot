#include "ros_helpers.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

geometry_msgs__msg__Twist msg_sub;
nav_msgs__msg__Odometry odometry_msg;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

void micro_ros_setup(char *ssid, char *pass, char *agent_ip, uint16_t agent_port, const char *node_name)
{
  IPAddress ip;
  ip.fromString(agent_ip);

  // micro-ROS UDP transport
  set_microros_wifi_transports(ssid, pass, ip, agent_port);

  int timeout_ms = 500;
  int attempts = 10;
  while (rmw_uros_ping_agent(timeout_ms, 1) != RCL_RET_OK)
  {
    Serial.print("Pinging Agent: ");
    Serial.print(agent_ip);
    Serial.println("...");

    attempts--;
    if (attempts == 0)
    {
      Serial.println("Could not connect to Agent. Restarting...");
      delay(1000);
      ESP.restart();
    }
  }

  Serial.println("Connected to Agent!");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, node_name, "", &support);
}

void ros_subscription_init(const char *topic_name, rclc_subscription_callback_t callback = NULL)
{
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      topic_name);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
      &executor,
      &subscriber,
      &msg_sub,
      callback,
      ON_NEW_DATA);
}

void ros_publisher_init(const char *topic_name)
{
  init_odom_msg();

  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      topic_name);
}

void init_odom_msg()
{
  // Initialize message memory
  nav_msgs__msg__Odometry__init(&odometry_msg);

  // Frame IDs
  rosidl_runtime_c__String__assign(
      &odometry_msg.header.frame_id,
      "odom");

  rosidl_runtime_c__String__assign(
      &odometry_msg.child_frame_id,
      "base_link");
}

void ros_subscription()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void ros_publish(const nav_msgs__msg__Odometry &msg)
{
  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

  if (ret != RCL_RET_OK)
  {
    Serial.print("Publish failed! Error code: ");
    Serial.println(ret);
  }
}

void ros_publishOdometry(Pose pose)
{
  odometry_msg.pose.pose.position.x = pose.x;
  odometry_msg.pose.pose.position.y = pose.y;
  odometry_msg.pose.pose.position.z = 0.0;

  odometry_msg.pose.pose.orientation.x = 0.0;
  odometry_msg.pose.pose.orientation.y = 0.0;
  float half = pose.theta * 0.5;
  odometry_msg.pose.pose.orientation.z = sin(half);
  odometry_msg.pose.pose.orientation.w = cos(half);

  printf("Publishing Odometry -> X: %f, Y: %f, Theta: %f\n", odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.orientation.z);

  ros_publish(odometry_msg);
}

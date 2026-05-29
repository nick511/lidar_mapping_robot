#include "ros_helpers.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <led_control_msgs/msg/led_status.h>
#include <geometry_msgs/msg/twist.h>

geometry_msgs__msg__Twist msg_sub;

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

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, node_name, "", &support);
}

/* void ros_publisher_init(const char *topic_name = "led_state")
{
  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(led_control_msgs, msg, LedStatus),
      topic_name);
} */

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

/* void ros_publish(led_control_msgs__msg__LedStatus led_msg)
{
  rcl_ret_t ret = rcl_publish(&publisher, &led_msg, NULL);

  if (ret != RCL_RET_OK)
  {
    Serial.print("Publish failed! Error code: ");
    Serial.println(ret);
  }
  else
  {
    printf("LED Status -> R: %d, G: %d, Y: %d\n", led_msg.red, led_msg.green, led_msg.yellow);
  }
} */

void ros_subscription()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

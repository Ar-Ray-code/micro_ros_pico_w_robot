#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "interface/interface.h"
#include <geometry_msgs/msg/twist.h>

#include "mechanum.h"

#define INTERFACE_WIFI

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// --- board ---
#define LED_A 18
#define LED_B 19
#define LED_C 20
#define LED_D 21

bool recieved_flag = false;

// --- micro-ROS ---
// Timer callback
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    (recieved_flag) ? (recieved_flag = false) : (stop());
}

// Twist subscription callback
void subscription_callback(const void *msgin)
{
    recieved_flag = true;
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float f_b = 0.0;
    float l_r = 0.0;
    if (msg->linear.x > 0.0)
        f_b = (msg->linear.x > SPEED_LIMIT) ? SPEED_LIMIT : msg->linear.x;
    else
        f_b = (msg->linear.x < -SPEED_LIMIT) ? -SPEED_LIMIT : msg->linear.x;

    if (msg->angular.z > 0.0)
        l_r = (msg->angular.z > SPEED_LIMIT) ? SPEED_LIMIT : msg->angular.z;
    else
        l_r = (msg->angular.z < -SPEED_LIMIT) ? -SPEED_LIMIT : msg->angular.z;

    gpio_put(LED_A, (f_b > 0.0) ? 1 : 0);
    gpio_put(LED_B, (f_b < 0.0) ? 1 : 0);
    gpio_put(LED_C, (l_r > 0.0) ? 1 : 0);
    gpio_put(LED_D, (l_r < 0.0) ? 1 : 0);

    omni(0, f_b, l_r);
    sleep_ms(100);

}

int main()
{
    gpio_init(LED_A);
    gpio_init(LED_B);
    gpio_init(LED_C);
    gpio_init(LED_D);

    gpio_set_dir(LED_A, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);
    gpio_set_dir(LED_C, GPIO_OUT);
    gpio_set_dir(LED_D, GPIO_OUT);

    add_pwm_slice_all();

    set_microros_wifi_transports("SSID", "PASS", "192.168.0.2", 2000);

    allocator = rcl_get_default_allocator();

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)  return ret;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    return 0;
}
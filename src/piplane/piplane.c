#include <stdio.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

// Replace with meaningful things
const uint GPIO[8] = {20, 13, 19, 11, 21, 12, 18, 10};
const int NUM_MOTORS = 8;
/**
 * Subscription call back for messages
*/
void subscription_callback(const void* msgin) {
    // Set the msgin to a Int16MultiArray
    std_msgs__msg__Int16MultiArray *msg = (std_msgs__msg__Int16MultiArray *) msgin;
    // set_duty_cycle((uint16_t *) msg->data.data);
    // This is where we should send the dshot packets
}

int main() {
    // // Get type support for Int16MultiArray (Change to correct type)
    // const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray);

    // rmw_uros_set_custom_transport(
	// 	true,
	// 	NULL,
	// 	pico_serial_transport_open,
	// 	pico_serial_transport_close,
	// 	pico_serial_transport_write,
	// 	pico_serial_transport_read
	// );

    // rcl_timer_t timer;
    // rcl_node_t node;
    // rcl_allocator_t allocator;
    // rclc_support_t support;
    // rclc_executor_t executor;

    // allocator = rcl_get_default_allocator();

    // // Wait for agent successful ping for 2 minutes.
    // const int timeout_ms = 1000; 
    // const uint8_t attempts = 120;

    // rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    // if (ret != RCL_RET_OK) {
    //     return ret; // Unreachable agent, exiting program.
    // }

    // // This is where we should init dshot

    // rclc_support_init(&support, 0, NULL, &allocator);
    
    // rclc_node_init_default(&node, "pico_node", "", &support);

    // // Define msg
    // std_msgs__msg__Int16MultiArray msg;

    // // Define msg data Int16 Sequence
    // rosidl_runtime_c__int16__Sequence msg_data;
    // msg_data.size = 0;
    // msg_data.capacity = 8;
    // int16_t msg_data_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // msg_data.data = msg_data_data;
    // msg.data = msg_data;

    // // Define msg layout MultiArray Layout
    // // FIXME: Change to correct message type
    // std_msgs__msg__MultiArrayLayout msg_layout;
    // std_msgs__msg__MultiArrayDimension__Sequence msg_layout_dim;
    // msg_layout.dim = msg_layout_dim;
    // msg_layout.data_offset = 0;
    // msg.layout = msg_layout;

    // // Subscribe to topic
    // ret = rclc_subscription_init_default(&subscriber, &node, type_support, "FIXME_TOPIC_NAME");
    // rclc_executor_init(&executor, &support.context, 1, &allocator);
    // // Choose callback for subscription
    // rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    // while (true) {
    //     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    // }

    return 0;
}

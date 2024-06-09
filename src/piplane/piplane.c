#include "dshot.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <stdbool.h>
#include <stdio.h>

rcl_subscription_t subscriber;
int16_t motor_throttles[4] = {0, 0, 0, 0};


void subscription_callback(const void *msgin) {
  gpio_put(15, 0);

  // Set the msgin to an Int16MultiArray
  // Expects array len 4, values 0-2047
  const std_msgs__msg__Int16MultiArray *msg =
      (const std_msgs__msg__Int16MultiArray *)msgin;

  for (int i = 0; i < 4 && i < msg->data.size; i++) {
    motor_throttles[i] = msg->data.data[i];
  }
}

#include "dshot.h"
#include "pico/stdlib.h"
#include <stdio.h>

struct dshot_controller controller0;

int main(void) {
  stdio_init_all();

  /* initialize controller 0 with DSHOT300 on pio0, state machine 0
   * with 1 channel on pin 2 */
  dshot_controller_init(&controller0, 300, pio0, 0, 2, 4);

  gpio_init(15);
  gpio_set_dir(15, GPIO_OUT);
  gpio_put(15, 1);

  const rosidl_message_type_support_t *type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray);

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    gpio_put(15, 0);
    return ret; // Unreachable agent, exiting program.
  }

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "motor_array", "", &support);

  // Define and initialize msg
  std_msgs__msg__Int16MultiArray msg;
  std_msgs__msg__MultiArrayLayout layout;
  std_msgs__msg__MultiArrayDimension layout_dim;

  // Initialize layout_dim
  layout_dim.label.data = "";
  layout_dim.label.size = 0;
  layout_dim.label.capacity = 0;
  layout_dim.size = 4;
  layout_dim.stride = 4;

  // Initialize layout
  std_msgs__msg__MultiArrayDimension__Sequence layout_dim_seq;
  layout_dim_seq.size = 1;
  layout_dim_seq.capacity = 1;
  layout_dim_seq.data = &layout_dim;

  layout.dim = layout_dim_seq;
  layout.data_offset = 0;

  // Initialize msg data
  msg.layout = layout;
  int16_t data[4] = {0, 0, 0, 0};
  msg.data.data = data;
  msg.data.size = 4;
  msg.data.capacity = 4;

  // Subscribe to topic from Pi, motor_array
  ret = rclc_subscription_init_default(&subscriber, &node, type_support,
                                       "/motor_array");
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Choose callback for subscription
  rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                 &subscription_callback, ON_NEW_DATA);
  int time;
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0.001));
    time = to_ms_since_boot(get_absolute_time());

    /* set motor throttles based on received array */

    for (int i = 0; i < 4; i++) {
      if (time > 5000) {
        dshot_throttle(&controller0, i, motor_throttles[i]);
      } else {
        dshot_throttle(&controller0, i, 0);
      }
    }

    dshot_loop(&controller0);
  }
  return 0;
}
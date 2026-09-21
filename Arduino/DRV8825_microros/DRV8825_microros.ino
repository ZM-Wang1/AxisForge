#include <AccelStepper.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Define stepper motor pins and interface
#define dirPin 13
#define stepPin 14
#define motorInterfaceType 1
#define microStep 32

// Create the stepper instance
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__Float32 speed_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float currentSpeed = 0.0;

// Callback for receiving speed in steps/sec
void speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float inputSpeed = msg->data;

  currentSpeed = inputSpeed;
  stepper.setSpeed(currentSpeed);

  // Optional debug output
  SerialUSB.print("Set speed to: ");
  SerialUSB.println(currentSpeed);
}

void setup() {
  SerialUSB.begin(115200);
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "stepper_controller_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "stepper/speed"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &speed_msg, &speed_callback, ON_NEW_DATA);

  stepper.setMaxSpeed(2000.0 * microStep);
  stepper.setSpeed(0);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_US_TO_NS(10));
  stepper.runSpeed();
}

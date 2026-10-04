#include <mbed.h>
#include <cmath>
#include <platform/mbed_power_mgmt.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Portenta H7 M7: D13 = PA10 (DIR), D14 = PA9 (TIM1 channel 2, STEP).
// Configure TIM1 directly. In Portenta core 4.6.0, pwmout_init_direct() falls
// back to pwmout_init(pin), which maps PA9 to HRTIM1 and ignores a custom map.
constexpr PinName STEP_PIN = PA_9;
constexpr PinName DIR_PIN = PA_10;
constexpr int STEP_PIN_FUNCTION =
  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF1_TIM1, 2, 0);
constexpr float MAX_STEPPER_SPEED = 64000.0f;
// Keep TIM1's 16-bit counter/prescaler within range even at a 480 MHz clock.
constexpr uint32_t MAX_STEP_PERIOD_US = 8000000;

gpio_t step_gpio;
gpio_t direction_gpio;
uint32_t step_timer_ticks_per_us = 0;
uint32_t step_prescaler_divider = 1;
uint32_t step_period_us = 0;
bool step_direction_positive = true;

void init_stepper_timer() {
  gpio_init_out_ex(&step_gpio, STEP_PIN, 0);
  gpio_init_out_ex(&direction_gpio, DIR_PIN, 0);
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();

  const uint32_t hclk = HAL_RCC_GetHCLKFreq();
  const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  // STM32H7 TIMPRE selects the APB timer clock multiplier.
  const uint32_t timer_clock = (RCC->CFGR & RCC_CFGR_TIMPRE)
    ? ((pclk2 >= hclk / 4) ? hclk : pclk2 * 4)
    : ((pclk2 == hclk) ? pclk2 : pclk2 * 2);
  step_timer_ticks_per_us = timer_clock / 1000000;

  // PWM mode 1 on channel 2, with buffered ARR/CCR updates. Keep the timer
  // stopped and STEP in GPIO-low mode until a nonzero ROS command arrives.
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM1->CCER = TIM_CCER_CC2E;
  TIM1->BDTR = TIM_BDTR_MOE;
}

void stop_stepper() {
  // Disconnect the timer and actively drive STEP low, even mid-period.
  gpio_init_out_ex(&step_gpio, STEP_PIN, 0);
  TIM1->CR1 &= ~TIM_CR1_CEN;
  step_period_us = 0;
}

void set_stepper_speed(float speed_steps_s) {
  if (!std::isfinite(speed_steps_s) || speed_steps_s == 0.0f) {
    stop_stepper();
    return;
  }

  double speed = std::fabs(static_cast<double>(speed_steps_s));
  if (speed > MAX_STEPPER_SPEED) {
    speed = MAX_STEPPER_SPEED;
  }
  const double requested_period = 1000000.0 / speed;
  if (requested_period > MAX_STEP_PERIOD_US) {
    // Rates below 0.125 pulses/s cannot be represented by this configuration.
    stop_stepper();
    return;
  }
  const uint32_t period_us = static_cast<uint32_t>(std::lround(requested_period));
  uint32_t divider = 1;
  while ((period_us - 1) / divider > 0xFFFF) {
    divider *= 2;
  }
  const uint32_t counter_period = (period_us - 1) / divider + 1;
  if (step_timer_ticks_per_us == 0 || step_timer_ticks_per_us * divider > 65536) {
    stop_stepper();
    return;
  }
  const bool positive = speed_steps_s > 0.0f;

  mbed::CriticalSectionLock lock;
  if (step_period_us != 0 && positive == step_direction_positive &&
      divider == step_prescaler_divider) {
    // Latch period and duty together at the next overflow, without restarting
    // the counter. Repeated ROS updates must not add pulses or reset phase.
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->ARR = counter_period - 1;
    TIM1->CCR2 = counter_period / 2;
    TIM1->CR1 &= ~TIM_CR1_UDIS;
  } else {
    // Startup, direction reversal, or a change of hardware prescaler.
    stop_stepper();
    wait_us(2);  // Preserve DIR hold time after the last STEP rising edge.
    gpio_write(&direction_gpio, positive ? 1 : 0);
    TIM1->PSC = step_timer_ticks_per_us * divider - 1;
    TIM1->ARR = counter_period - 1;
    TIM1->CCR2 = counter_period / 2;
    TIM1->CNT = 0;
    TIM1->EGR = TIM_EGR_UG;
    wait_us(2);  // DRV8825 DIR setup time is at least 650 ns.
    pin_function(STEP_PIN, STEP_PIN_FUNCTION);
    TIM1->CR1 |= TIM_CR1_CEN;
  }
  // At the maximum rate both STEP high/low exceed the required 1.9 us.
  step_period_us = period_us;
  step_prescaler_divider = divider;
  step_direction_positive = positive;
}

// Micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__Float32 speed_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
bool ros_ready = false;

bool check_ros_init(rcl_ret_t result) {
  if (result == RCL_RET_OK) {
    return true;
  }
  stop_stepper();
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);  // Portenta RGB LEDs are active-low.
  return false;
}

// Callback accepts signed motor STEP pulses per second.
void speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  // The host calibration already expresses speed in STEP pulses/s, including
  // the driver's microstepping. Do not multiply by the microstep setting again.
  set_stepper_speed(msg->data);
  digitalWrite(LEDG, LOW);  // At least one speed command reached the board.
  // USB serial is reserved for micro-ROS; do not print debug text here.
}

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);  // Blue while initializing micro-ROS.
  init_stepper_timer();
  // TIM1 must continue running when USB/ROS waits allow the CPU to sleep.
  sleep_manager_lock_deep_sleep();
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  if (!check_ros_init(rclc_support_init(&support, 0, NULL, &allocator))) return;
  if (!check_ros_init(rclc_node_init_default(&node, "stepper_controller_node", "", &support))) return;

  if (!check_ros_init(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "stepper/speed"
  ))) return;

  if (!check_ros_init(rclc_executor_init(&executor, &support.context, 1, &allocator))) return;
  if (!check_ros_init(rclc_executor_add_subscription(
      &executor, &subscriber, &speed_msg, &speed_callback, ON_NEW_DATA))) return;
  ros_ready = true;
  digitalWrite(LEDB, HIGH);
}

void loop() {
  if (!ros_ready) {
    delay(10);
    return;
  }
  // TIM1 generates STEP independently, including while this call blocks.
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

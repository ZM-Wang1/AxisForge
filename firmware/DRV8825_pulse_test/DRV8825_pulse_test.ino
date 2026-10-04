#include <Arduino.h>
#include <platform/mbed_power_mgmt.h>

// Standalone Portenta H7 M7 diagnostic: no micro-ROS, AccelStepper, or TIM1.
// Keep the production wiring: D14/PA9 -> STEP, D13/PA10 -> DIR.
// Uploading does not start motion. Send 'g' over USB serial to run one test.
constexpr pin_size_t STEP_PIN = 14;
constexpr pin_size_t DIR_PIN = 13;
constexpr uint32_t HALF_PERIOD_US = 2500;  // About 200 STEP pulses/s, no x32.
constexpr uint32_t TEST_DURATION_US = 20000000;  // Stop after 20 seconds.

void print_help() {
  Serial.println("GPIO pulse test: g = run 200 pulses/s for 20 s; s = stop; ? = help");
  Serial.println("No automatic restart. Timing reports software writes, not motor motion.");
}

void run_pulse_test() {
  Serial.println("START: 200 pulses/s, 20 s. No serial output until STEP stops.");
  Serial.flush();  // Complete console output before starting the pulse train.
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(5);

  const uint32_t started_us = micros();
  uint32_t last_edge_us = started_us;
  uint32_t max_gap_us = 0;
  uint32_t pulses = 0;
  uint32_t readback_errors = 0;
  bool stopped_by_user = false;

  while (static_cast<uint32_t>(micros() - started_us) < TEST_DURATION_US) {
    // Bounded, nonblocking input. Do not parse strings or print during motion.
    for (int i = 0; i < 16 && Serial.available() > 0; ++i) {
      const int command = Serial.read();
      if (command == 's' || command == 'S') {
        stopped_by_user = true;
      }
      // Ignore 'g' while running, so queued commands cannot restart this test.
    }
    if (stopped_by_user ||
        static_cast<uint32_t>(micros() - started_us) >= TEST_DURATION_US) {
      break;
    }

    digitalWrite(STEP_PIN, HIGH);
    const uint32_t edge_us = micros();
    const uint32_t gap_us = edge_us - last_edge_us;
    if (gap_us > max_gap_us) max_gap_us = gap_us;
    last_edge_us = edge_us;
    ++pulses;
    // digitalRead on this configured output reads its pad without changing mode.
    if (digitalRead(STEP_PIN) != HIGH) ++readback_errors;
    delayMicroseconds(HALF_PERIOD_US);
    digitalWrite(STEP_PIN, LOW);
    if (digitalRead(STEP_PIN) != LOW) ++readback_errors;
    delayMicroseconds(HALF_PERIOD_US);
  }

  // Stop before any serial reporting; a disconnected host cannot prolong motion
  // just because console writes block. There is no watchdog for a CPU hang.
  digitalWrite(STEP_PIN, LOW);
  const uint32_t finished_us = micros();
  const uint32_t tail_gap_us = finished_us - last_edge_us;
  if (tail_gap_us > max_gap_us) max_gap_us = tail_gap_us;
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  // Also discard commands received during the final pulse's delays.
  while (Serial.available() > 0) Serial.read();

  Serial.print("DONE reason=");
  Serial.print(stopped_by_user ? "user_stop" : "duration");
  Serial.print(" pulses=");
  Serial.print(pulses);
  Serial.print(" elapsed_ms=");
  Serial.print((finished_us - started_us) / 1000);
  Serial.print(" max_gap_us=");
  Serial.print(max_gap_us);
  Serial.print(" readback_errors=");
  Serial.println(readback_errors);
  print_help();
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);  // Blue: idle. Green: inside the timed test.
  sleep_manager_lock_deep_sleep();
  Serial.begin(115200);
  while (!Serial) delay(10);
  print_help();
}

void loop() {
  if (Serial.available() > 0) {
    const int command = Serial.read();
    if (command == 'g' || command == 'G') {
      run_pulse_test();
    } else if (command == '?') {
      print_help();
    }
  }
}

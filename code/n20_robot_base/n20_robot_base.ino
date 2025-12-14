/*
 * N20 Robot Base Control System
 *
 * A modular robot control system for N20 motors with quadrature encoders
 * Features:
 * - PID-based speed control
 * - Serial command interface
 * - Configurable parameters (wheel size, gear ratio, PID gains)
 * - Movement commands (forward, backward, turn, speed control)
 *
 * Hardware:
 * - Arduino Uno
 * - L298N Motor Driver
 * - 2x N20 Motors with Quadrature Encoders
 *
 * Required Libraries:
 * - Encoder (https://www.pjrc.com/teensy/td_libs_Encoder.html)
 *   Install via Library Manager: "Encoder by Paul Stoffregen"
 */

#include "pin_map.h"
#include "config.h"
#include "MotorController.h"
#include "SerialCommands.h"

// Global configuration instance
RobotConfig config;

// Create motor controllers
MotorController leftMotor(
  LEFT_MOTOR_ENA,
  LEFT_MOTOR_IN1,
  LEFT_MOTOR_IN2,
  LEFT_ENCODER_A,
  LEFT_ENCODER_B
);

MotorController rightMotor(
  RIGHT_MOTOR_ENB,
  RIGHT_MOTOR_IN3,
  RIGHT_MOTOR_IN4,
  RIGHT_ENCODER_A,
  RIGHT_ENCODER_B
);

// Create serial command interface
SerialCommands serialCmd(&leftMotor, &rightMotor);

// Timing variables
unsigned long lastControlUpdate = 0;

void setup() {
  // Initialize serial communication
  serialCmd.begin(115200);

  // Initialize motors
  leftMotor.begin();
  rightMotor.begin();

  // Set initial PID gains from config
  leftMotor.setPIDGains(config.leftKp, config.leftKi, config.leftKd);
  rightMotor.setPIDGains(config.rightKp, config.rightKi, config.rightKd);

  Serial.println(F("System initialized successfully!"));
  Serial.println(F("Type 'help' for available commands.\n"));
}

void loop() {
  // Process serial commands
  serialCmd.processSerial();

  // Update motor control at fixed interval
  unsigned long currentTime = millis();
  if (currentTime - lastControlUpdate >= config.controlLoopInterval) {
    leftMotor.updateControl();
    rightMotor.updateControl();
    lastControlUpdate = currentTime;
  }
}

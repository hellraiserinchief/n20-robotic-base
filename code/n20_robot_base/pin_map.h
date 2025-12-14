#ifndef PIN_MAP_H
#define PIN_MAP_H

// L298N Motor Driver Pins
// Left Motor
#define LEFT_MOTOR_ENA    5    // PWM pin for left motor speed (Enable A)
#define LEFT_MOTOR_IN1    7    // Direction control 1
#define LEFT_MOTOR_IN2    8    // Direction control 2

// Right Motor
#define RIGHT_MOTOR_ENB   6    // PWM pin for right motor speed (Enable B)
#define RIGHT_MOTOR_IN3   9    // Direction control 3
#define RIGHT_MOTOR_IN4   10   // Direction control 4

// Encoder Pins (using interrupt-capable pins)
// Left Encoder
#define LEFT_ENCODER_A    2    // Interrupt pin (INT0)
#define LEFT_ENCODER_B    4    // Regular digital pin

// Right Encoder
#define RIGHT_ENCODER_A   3    // Interrupt pin (INT1)
#define RIGHT_ENCODER_B   11   // Regular digital pin

// Optional: Additional sensors can be added here
// #define ULTRASONIC_TRIG  12
// #define ULTRASONIC_ECHO  13
// #define IMU_SDA          A4
// #define IMU_SCL          A5

#endif // PIN_MAP_H

#ifndef CONFIG_H
#define CONFIG_H

// Robot Physical Parameters
struct RobotConfig {
  // Wheel parameters
  float wheelDiameter = 65.0;        // Wheel diameter in mm
  float wheelBase = 150.0;           // Distance between wheels in mm

  // Motor and encoder parameters
  int encoderPPR = 480;              // Pulses per revolution (N20 with quad encoder)
  float gearRatio = 100.0;           // Gear ratio (typical for N20 motors)

  // PID parameters for left motor
  float leftKp = 2.0;
  float leftKi = 0.5;
  float leftKd = 0.1;

  // PID parameters for right motor
  float rightKp = 2.0;
  float rightKi = 0.5;
  float rightKd = 0.1;

  // Speed limits
  int maxPWM = 255;                  // Maximum PWM value
  int minPWM = 0;                    // Minimum PWM value
  float maxSpeed = 100.0;            // Maximum speed in RPM

  // Control loop timing
  unsigned long controlLoopInterval = 50;  // Control loop interval in ms (20Hz)
};

// Global configuration instance
extern RobotConfig config;

#endif // CONFIG_H

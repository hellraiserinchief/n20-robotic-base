#ifndef CONFIG_H
#define CONFIG_H

// Robot Physical Parameters
struct RobotConfig {
  // Wheel parameters
  float wheelDiameter = 65.0;        // Wheel diameter in mm
  float wheelBase = 150.0;           // Distance between left and right wheels in mm

  // Robot geometry (three-point contact system)
  float casterOffset = 75.0;         // Distance from drive wheel axis to caster (mm)
                                     // Positive = caster in front, Negative = caster in back
  float wheelAxisOffset = 0.0;       // Offset of wheel axis from robot center (mm)
                                     // Positive = wheels forward of center

  // Turning calibration
  float turningCalibration = 1.0;    // Multiplier to correct turning accuracy (0.8-1.2)
                                     // <1.0 if robot turns too much, >1.0 if too little

  // WASD control mode
  float wasdSpeed = 30.0;            // Default speed for WASD mode (RPM)
  float wasdTurnSpeed = 25.0;        // Default turning speed for WASD mode (RPM)

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

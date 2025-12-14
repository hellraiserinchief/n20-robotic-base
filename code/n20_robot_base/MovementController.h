#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "MotorController.h"
#include "config.h"

enum MovementMode {
  MODE_SPEED,      // Continuous speed control
  MODE_DISTANCE,   // Move specific distance
  MODE_ANGLE       // Turn specific angle
};

enum MovementState {
  MOVEMENT_IDLE,
  MOVEMENT_ACTIVE,
  MOVEMENT_COMPLETE
};

class MovementController {
private:
  MotorController* leftMotor;
  MotorController* rightMotor;

  MovementMode currentMode;
  MovementState currentState;

  // Target position tracking
  long targetEncoderTicks;
  long startLeftEncoder;
  long startRightEncoder;

  float movementSpeed;  // Speed to use for position-based movements (RPM)

public:
  MovementController(MotorController* left, MotorController* right)
    : leftMotor(left), rightMotor(right),
      currentMode(MODE_SPEED), currentState(MOVEMENT_IDLE),
      targetEncoderTicks(0), startLeftEncoder(0), startRightEncoder(0),
      movementSpeed(30.0) {}  // Default 30 RPM for position movements

  void setMovementSpeed(float speedRPM) {
    movementSpeed = speedRPM;
  }

  float getMovementSpeed() {
    return movementSpeed;
  }

  // Convert distance in mm to encoder ticks
  long distanceToTicks(float distanceMM) {
    // Distance per wheel revolution = π * diameter
    float distancePerRev = PI * config.wheelDiameter;

    // Revolutions needed
    float revolutions = distanceMM / distancePerRev;

    // Encoder ticks = revolutions * PPR * gear ratio
    long ticks = (long)(revolutions * config.encoderPPR * config.gearRatio);

    return ticks;
  }

  // Convert angle in degrees to encoder ticks (for differential drive)
  long angleToTicks(float angleDegrees) {
    // Arc length each wheel travels for rotation
    // For differential drive: arc = (wheelbase / 2) * angle_in_radians
    float angleRadians = angleDegrees * PI / 180.0;
    float arcLength = (config.wheelBase / 2.0) * angleRadians;

    // Apply turning calibration factor to account for:
    // - Caster wheel drag
    // - Center of mass offset
    // - Wheel slippage
    // - Three-point contact geometry
    arcLength *= config.turningCalibration;

    // Convert arc length to encoder ticks
    long ticks = distanceToTicks(arcLength);

    return ticks;
  }

  // Move forward by distance (in mm)
  void moveDistanceForward(float distanceMM) {
    targetEncoderTicks = distanceToTicks(distanceMM);
    startLeftEncoder = leftMotor->getEncoderCount();
    startRightEncoder = rightMotor->getEncoderCount();

    leftMotor->setTargetSpeed(movementSpeed);
    rightMotor->setTargetSpeed(movementSpeed);

    currentMode = MODE_DISTANCE;
    currentState = MOVEMENT_ACTIVE;
  }

  // Move backward by distance (in mm)
  void moveDistanceBackward(float distanceMM) {
    targetEncoderTicks = distanceToTicks(distanceMM);
    startLeftEncoder = leftMotor->getEncoderCount();
    startRightEncoder = rightMotor->getEncoderCount();

    leftMotor->setTargetSpeed(-movementSpeed);
    rightMotor->setTargetSpeed(-movementSpeed);

    currentMode = MODE_DISTANCE;
    currentState = MOVEMENT_ACTIVE;
  }

  // Turn by angle (positive = right, negative = left)
  void turnAngle(float angleDegrees) {
    targetEncoderTicks = angleToTicks(abs(angleDegrees));
    startLeftEncoder = leftMotor->getEncoderCount();
    startRightEncoder = rightMotor->getEncoderCount();

    if (angleDegrees > 0) {
      // Turn right: left forward, right backward
      leftMotor->setTargetSpeed(movementSpeed);
      rightMotor->setTargetSpeed(-movementSpeed);
    } else {
      // Turn left: left backward, right forward
      leftMotor->setTargetSpeed(-movementSpeed);
      rightMotor->setTargetSpeed(movementSpeed);
    }

    currentMode = MODE_ANGLE;
    currentState = MOVEMENT_ACTIVE;
  }

  // Set speed mode (continuous)
  void setSpeedMode() {
    currentMode = MODE_SPEED;
    currentState = MOVEMENT_IDLE;
  }

  // Update movement state (call in main loop)
  void update() {
    if (currentState != MOVEMENT_ACTIVE) {
      return;
    }

    if (currentMode == MODE_DISTANCE) {
      // Check if target distance reached (average of both wheels)
      long leftTraveled = abs(leftMotor->getEncoderCount() - startLeftEncoder);
      long rightTraveled = abs(rightMotor->getEncoderCount() - startRightEncoder);
      long avgTraveled = (leftTraveled + rightTraveled) / 2;

      if (avgTraveled >= targetEncoderTicks) {
        leftMotor->stop();
        rightMotor->stop();
        currentState = MOVEMENT_COMPLETE;
      }
    }
    else if (currentMode == MODE_ANGLE) {
      // Check if target angle reached (average of both wheels)
      long leftTraveled = abs(leftMotor->getEncoderCount() - startLeftEncoder);
      long rightTraveled = abs(rightMotor->getEncoderCount() - startRightEncoder);
      long avgTraveled = (leftTraveled + rightTraveled) / 2;

      if (avgTraveled >= targetEncoderTicks) {
        leftMotor->stop();
        rightMotor->stop();
        currentState = MOVEMENT_COMPLETE;
      }
    }
  }

  // Check if movement is complete
  bool isMovementComplete() {
    return currentState == MOVEMENT_COMPLETE;
  }

  // Check if movement is active
  bool isMovementActive() {
    return currentState == MOVEMENT_ACTIVE;
  }

  // Stop current movement
  void stopMovement() {
    leftMotor->stop();
    rightMotor->stop();
    currentState = MOVEMENT_IDLE;
    currentMode = MODE_SPEED;
  }

  // Get current movement state
  MovementState getState() {
    return currentState;
  }

  MovementMode getMode() {
    return currentMode;
  }

  // Get progress (0.0 to 1.0)
  float getProgress() {
    if (currentState != MOVEMENT_ACTIVE) {
      return 0.0;
    }

    long leftTraveled = abs(leftMotor->getEncoderCount() - startLeftEncoder);
    long rightTraveled = abs(rightMotor->getEncoderCount() - startRightEncoder);
    long avgTraveled = (leftTraveled + rightTraveled) / 2;

    if (targetEncoderTicks == 0) {
      return 0.0;
    }

    float progress = (float)avgTraveled / (float)targetEncoderTicks;
    return min(progress, 1.0);
  }
};

#endif // MOVEMENT_CONTROLLER_H

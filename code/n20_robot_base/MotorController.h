#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Encoder.h>
#include "PIDController.h"
#include "config.h"

enum MotorDirection {
  FORWARD,
  BACKWARD,
  BRAKE
};

class MotorController {
private:
  // Pin assignments
  int enablePin;
  int in1Pin;
  int in2Pin;

  // Encoder
  Encoder* encoder;

  // PID controller
  PIDController pid;

  // Speed tracking
  float targetSpeed;        // Target speed in RPM
  float currentSpeed;       // Current speed in RPM
  long lastEncoderCount;
  unsigned long lastSpeedUpdate;

  // Direction
  MotorDirection currentDirection;

public:
  MotorController(int enPin, int in1, int in2, int encA, int encB)
    : enablePin(enPin), in1Pin(in1), in2Pin(in2),
      targetSpeed(0), currentSpeed(0), lastEncoderCount(0),
      lastSpeedUpdate(0), currentDirection(BRAKE) {

    encoder = new Encoder(encA, encB);
    pid.setOutputLimits(-255, 255);
  }

  void begin() {
    pinMode(enablePin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);

    stop();
    encoder->write(0);
    lastSpeedUpdate = millis();
  }

  void setPIDGains(float kp, float ki, float kd) {
    pid.setGains(kp, ki, kd);
  }

  void setTargetSpeed(float speedRPM) {
    targetSpeed = speedRPM;
    if (targetSpeed > 0) {
      currentDirection = FORWARD;
    } else if (targetSpeed < 0) {
      currentDirection = BACKWARD;
    }
  }

  void updateSpeed() {
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - lastSpeedUpdate;

    if (dt >= 50) { // Update every 50ms
      long currentCount = encoder->read();
      long deltaCount = currentCount - lastEncoderCount;

      // Calculate RPM
      // RPM = (deltaCount / PPR) * (60000 / dt) / gearRatio
      float pulsesPerRev = config.encoderPPR * config.gearRatio;
      currentSpeed = (deltaCount / pulsesPerRev) * (60000.0 / dt);

      lastEncoderCount = currentCount;
      lastSpeedUpdate = currentTime;
    }
  }

  void updateControl() {
    updateSpeed();

    // Compute PID output
    float pidOutput = pid.compute(abs(targetSpeed), abs(currentSpeed));

    // Apply direction and PWM
    if (targetSpeed == 0) {
      stop();
    } else {
      int pwmValue = constrain(abs(pidOutput), 0, 255);

      if (targetSpeed > 0) {
        setDirection(FORWARD);
      } else {
        setDirection(BACKWARD);
      }

      analogWrite(enablePin, pwmValue);
    }
  }

  void setRawPWM(int pwm, MotorDirection dir) {
    setDirection(dir);
    pwm = constrain(abs(pwm), 0, 255);
    analogWrite(enablePin, pwm);
  }

  void stop() {
    setDirection(BRAKE);
    analogWrite(enablePin, 0);
    targetSpeed = 0;
    pid.reset();
  }

  float getCurrentSpeed() {
    return currentSpeed;
  }

  float getTargetSpeed() {
    return targetSpeed;
  }

  long getEncoderCount() {
    return encoder->read();
  }

  void resetEncoder() {
    encoder->write(0);
    lastEncoderCount = 0;
  }

private:
  void setDirection(MotorDirection dir) {
    currentDirection = dir;

    switch (dir) {
      case FORWARD:
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        break;
      case BACKWARD:
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        break;
      case BRAKE:
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        break;
    }
  }
};

#endif // MOTOR_CONTROLLER_H

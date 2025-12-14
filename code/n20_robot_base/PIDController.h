#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
  float kp, ki, kd;
  float integral;
  float previousError;
  float outputMin, outputMax;
  unsigned long lastTime;
  bool firstRun;

public:
  PIDController(float p = 1.0, float i = 0.0, float d = 0.0)
    : kp(p), ki(i), kd(d), integral(0), previousError(0),
      outputMin(-255), outputMax(255), lastTime(0), firstRun(true) {}

  void setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  void setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
  }

  float compute(float setpoint, float measured) {
    unsigned long currentTime = millis();

    if (firstRun) {
      lastTime = currentTime;
      previousError = 0;
      integral = 0;
      firstRun = false;
      return 0;
    }

    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    if (dt <= 0) dt = 0.001; // Prevent division by zero

    // Calculate error
    float error = setpoint - measured;

    // Proportional term
    float pTerm = kp * error;

    // Integral term with anti-windup
    integral += error * dt;
    float iTerm = ki * integral;

    // Derivative term
    float derivative = (error - previousError) / dt;
    float dTerm = kd * derivative;

    // Calculate output
    float output = pTerm + iTerm + dTerm;

    // Apply output limits
    if (output > outputMax) {
      output = outputMax;
      // Anti-windup: prevent integral from growing when saturated
      integral -= error * dt;
    } else if (output < outputMin) {
      output = outputMin;
      // Anti-windup
      integral -= error * dt;
    }

    // Save values for next iteration
    previousError = error;
    lastTime = currentTime;

    return output;
  }

  void reset() {
    integral = 0;
    previousError = 0;
    firstRun = true;
  }

  void getGains(float &p, float &i, float &d) {
    p = kp;
    i = ki;
    d = kd;
  }
};

#endif // PID_CONTROLLER_H

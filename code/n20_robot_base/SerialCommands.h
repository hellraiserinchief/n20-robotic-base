#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include "MotorController.h"
#include "MovementController.h"
#include "config.h"

class SerialCommands {
private:
  MotorController* leftMotor;
  MotorController* rightMotor;
  MovementController* movementCtrl;
  String inputBuffer;

public:
  SerialCommands(MotorController* left, MotorController* right, MovementController* movement)
    : leftMotor(left), rightMotor(right), movementCtrl(movement), inputBuffer("") {}

  void begin(long baudRate = 115200) {
    Serial.begin(baudRate);
    Serial.println(F("================================="));
    Serial.println(F("N20 Robot Base Control System"));
    Serial.println(F("================================="));
    printHelp();
  }

  void processSerial() {
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          processCommand(inputBuffer);
          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }
  }

private:
  void processCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.startsWith("help")) {
      printHelp();
    }
    // Movement commands
    else if (cmd.startsWith("forward ")) {
      float speed = cmd.substring(8).toFloat();
      moveForward(speed);
    }
    else if (cmd.startsWith("backward ")) {
      float speed = cmd.substring(9).toFloat();
      moveBackward(speed);
    }
    else if (cmd.startsWith("left ")) {
      float speed = cmd.substring(5).toFloat();
      turnLeft(speed);
    }
    else if (cmd.startsWith("right ")) {
      float speed = cmd.substring(6).toFloat();
      turnRight(speed);
    }
    else if (cmd.startsWith("speed ")) {
      float speed = cmd.substring(6).toFloat();
      setSpeed(speed);
    }
    else if (cmd.startsWith("stop")) {
      stop();
    }
    // Position-based movement commands
    else if (cmd.startsWith("go ")) {
      parseGoCommand(cmd.substring(3));
    }
    else if (cmd.startsWith("turn ")) {
      parseTurnCommand(cmd.substring(5));
    }
    else if (cmd.startsWith("movespeed ")) {
      setMovementSpeed(cmd.substring(10).toFloat());
    }
    // PID configuration
    else if (cmd.startsWith("pid left ")) {
      setPIDLeft(cmd.substring(9));
    }
    else if (cmd.startsWith("pid right ")) {
      setPIDRight(cmd.substring(10));
    }
    // Robot configuration
    else if (cmd.startsWith("wheel ")) {
      setWheelDiameter(cmd.substring(6).toFloat());
    }
    else if (cmd.startsWith("wheelbase ")) {
      setWheelBase(cmd.substring(10).toFloat());
    }
    else if (cmd.startsWith("gear ")) {
      setGearRatio(cmd.substring(5).toFloat());
    }
    else if (cmd.startsWith("ppr ")) {
      setEncoderPPR(cmd.substring(4).toInt());
    }
    else if (cmd.startsWith("caster ")) {
      setCasterOffset(cmd.substring(7).toFloat());
    }
    else if (cmd.startsWith("turncal ")) {
      setTurningCalibration(cmd.substring(8).toFloat());
    }
    // Status and diagnostics
    else if (cmd.startsWith("status")) {
      printStatus();
    }
    else if (cmd.startsWith("config")) {
      printConfig();
    }
    else if (cmd.startsWith("reset encoders")) {
      resetEncoders();
    }
    else {
      Serial.println(F("Unknown command. Type 'help' for available commands."));
    }
  }

  void printHelp() {
    Serial.println(F("\n--- Speed-Based Movement ---"));
    Serial.println(F("forward <speed>    - Move forward at speed (RPM)"));
    Serial.println(F("backward <speed>   - Move backward at speed (RPM)"));
    Serial.println(F("left <speed>       - Turn left at speed (RPM)"));
    Serial.println(F("right <speed>      - Turn right at speed (RPM)"));
    Serial.println(F("speed <speed>      - Set both motors to speed (RPM)"));
    Serial.println(F("stop               - Stop all motors"));
    Serial.println(F("\n--- Position-Based Movement ---"));
    Serial.println(F("go <dist> <unit>   - Move forward/back (e.g., 'go 50 cm', 'go -1 m')"));
    Serial.println(F("turn <angle>       - Turn angle in degrees (+ right, - left)"));
    Serial.println(F("movespeed <speed>  - Set speed for position movements (RPM)"));
    Serial.println(F("\n--- PID Configuration ---"));
    Serial.println(F("pid left <kp> <ki> <kd>   - Set left motor PID gains"));
    Serial.println(F("pid right <kp> <ki> <kd>  - Set right motor PID gains"));
    Serial.println(F("\n--- Robot Configuration ---"));
    Serial.println(F("wheel <diameter>   - Set wheel diameter (mm)"));
    Serial.println(F("wheelbase <width>  - Set wheelbase width (mm)"));
    Serial.println(F("gear <ratio>       - Set gear ratio"));
    Serial.println(F("ppr <pulses>       - Set encoder pulses per rev"));
    Serial.println(F("caster <offset>    - Set caster offset from wheel axis (mm)"));
    Serial.println(F("turncal <factor>   - Set turning calibration (0.8-1.2)"));
    Serial.println(F("\n--- Status & Diagnostics ---"));
    Serial.println(F("status             - Print motor status"));
    Serial.println(F("config             - Print configuration"));
    Serial.println(F("reset encoders     - Reset encoder counts"));
    Serial.println(F("help               - Show this help"));
    Serial.println();
  }

  // Movement functions
  void moveForward(float speed) {
    leftMotor->setTargetSpeed(speed);
    rightMotor->setTargetSpeed(speed);
    Serial.print(F("Moving forward at "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  void moveBackward(float speed) {
    leftMotor->setTargetSpeed(-speed);
    rightMotor->setTargetSpeed(-speed);
    Serial.print(F("Moving backward at "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  void turnLeft(float speed) {
    leftMotor->setTargetSpeed(-speed);
    rightMotor->setTargetSpeed(speed);
    Serial.print(F("Turning left at "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  void turnRight(float speed) {
    leftMotor->setTargetSpeed(speed);
    rightMotor->setTargetSpeed(-speed);
    Serial.print(F("Turning right at "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  void setSpeed(float speed) {
    leftMotor->setTargetSpeed(speed);
    rightMotor->setTargetSpeed(speed);
    Serial.print(F("Set speed to "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  void stop() {
    movementCtrl->stopMovement();
    leftMotor->stop();
    rightMotor->stop();
    Serial.println(F("Stopped"));
  }

  // PID configuration
  void setPIDLeft(String params) {
    float kp, ki, kd;
    if (parseThreeFloats(params, kp, ki, kd)) {
      config.leftKp = kp;
      config.leftKi = ki;
      config.leftKd = kd;
      leftMotor->setPIDGains(kp, ki, kd);
      Serial.print(F("Left PID: Kp="));
      Serial.print(kp);
      Serial.print(F(" Ki="));
      Serial.print(ki);
      Serial.print(F(" Kd="));
      Serial.println(kd);
    } else {
      Serial.println(F("Error: Use format 'pid left <kp> <ki> <kd>'"));
    }
  }

  void setPIDRight(String params) {
    float kp, ki, kd;
    if (parseThreeFloats(params, kp, ki, kd)) {
      config.rightKp = kp;
      config.rightKi = ki;
      config.rightKd = kd;
      rightMotor->setPIDGains(kp, ki, kd);
      Serial.print(F("Right PID: Kp="));
      Serial.print(kp);
      Serial.print(F(" Ki="));
      Serial.print(ki);
      Serial.print(F(" Kd="));
      Serial.println(kd);
    } else {
      Serial.println(F("Error: Use format 'pid right <kp> <ki> <kd>'"));
    }
  }

  // Robot configuration
  void setWheelDiameter(float diameter) {
    config.wheelDiameter = diameter;
    Serial.print(F("Wheel diameter set to "));
    Serial.print(diameter);
    Serial.println(F(" mm"));
  }

  void setWheelBase(float width) {
    config.wheelBase = width;
    Serial.print(F("Wheelbase set to "));
    Serial.print(width);
    Serial.println(F(" mm"));
  }

  void setGearRatio(float ratio) {
    config.gearRatio = ratio;
    Serial.print(F("Gear ratio set to "));
    Serial.println(ratio);
  }

  void setEncoderPPR(int ppr) {
    config.encoderPPR = ppr;
    Serial.print(F("Encoder PPR set to "));
    Serial.println(ppr);
  }

  void setCasterOffset(float offset) {
    config.casterOffset = offset;
    Serial.print(F("Caster offset set to "));
    Serial.print(offset);
    Serial.println(F(" mm"));
  }

  void setTurningCalibration(float factor) {
    config.turningCalibration = factor;
    Serial.print(F("Turning calibration set to "));
    Serial.println(factor);
  }

  // Status and diagnostics
  void printStatus() {
    Serial.println(F("\n--- Motor Status ---"));
    Serial.print(F("Left Motor  - Target: "));
    Serial.print(leftMotor->getTargetSpeed());
    Serial.print(F(" RPM, Current: "));
    Serial.print(leftMotor->getCurrentSpeed());
    Serial.print(F(" RPM, Encoder: "));
    Serial.println(leftMotor->getEncoderCount());

    Serial.print(F("Right Motor - Target: "));
    Serial.print(rightMotor->getTargetSpeed());
    Serial.print(F(" RPM, Current: "));
    Serial.print(rightMotor->getCurrentSpeed());
    Serial.print(F(" RPM, Encoder: "));
    Serial.println(rightMotor->getEncoderCount());
    Serial.println();
  }

  void printConfig() {
    Serial.println(F("\n--- Robot Configuration ---"));
    Serial.print(F("Wheel Diameter: "));
    Serial.print(config.wheelDiameter);
    Serial.println(F(" mm"));
    Serial.print(F("Wheelbase: "));
    Serial.print(config.wheelBase);
    Serial.println(F(" mm"));
    Serial.print(F("Caster Offset: "));
    Serial.print(config.casterOffset);
    Serial.println(F(" mm"));
    Serial.print(F("Gear Ratio: "));
    Serial.println(config.gearRatio);
    Serial.print(F("Encoder PPR: "));
    Serial.println(config.encoderPPR);
    Serial.print(F("Turning Calibration: "));
    Serial.println(config.turningCalibration);

    Serial.println(F("\n--- PID Gains ---"));
    Serial.print(F("Left Motor  - Kp: "));
    Serial.print(config.leftKp);
    Serial.print(F(", Ki: "));
    Serial.print(config.leftKi);
    Serial.print(F(", Kd: "));
    Serial.println(config.leftKd);

    Serial.print(F("Right Motor - Kp: "));
    Serial.print(config.rightKp);
    Serial.print(F(", Ki: "));
    Serial.print(config.rightKi);
    Serial.print(F(", Kd: "));
    Serial.println(config.rightKd);
    Serial.println();
  }

  void resetEncoders() {
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
    Serial.println(F("Encoders reset"));
  }

  // Position-based movement functions
  void parseGoCommand(String params) {
    params.trim();
    int spaceIndex = params.indexOf(' ');

    if (spaceIndex == -1) {
      Serial.println(F("Error: Use format 'go <distance> <unit>' (e.g., 'go 50 cm' or 'go 1.5 m')"));
      return;
    }

    float distance = params.substring(0, spaceIndex).toFloat();
    String unit = params.substring(spaceIndex + 1);
    unit.trim();
    unit.toLowerCase();

    float distanceMM = 0;

    if (unit == "cm") {
      distanceMM = distance * 10.0;  // Convert cm to mm
    } else if (unit == "m") {
      distanceMM = distance * 1000.0;  // Convert m to mm
    } else if (unit == "mm") {
      distanceMM = distance;
    } else {
      Serial.println(F("Error: Unit must be 'cm', 'm', or 'mm'"));
      return;
    }

    if (distanceMM > 0) {
      movementCtrl->moveDistanceForward(distanceMM);
      Serial.print(F("Moving forward "));
      Serial.print(distance);
      Serial.print(F(" "));
      Serial.println(unit);
    } else if (distanceMM < 0) {
      movementCtrl->moveDistanceBackward(abs(distanceMM));
      Serial.print(F("Moving backward "));
      Serial.print(abs(distance));
      Serial.print(F(" "));
      Serial.println(unit);
    } else {
      Serial.println(F("Distance is zero"));
    }
  }

  void parseTurnCommand(String params) {
    params.trim();
    float angle = params.toFloat();

    if (angle == 0) {
      Serial.println(F("Angle is zero"));
      return;
    }

    movementCtrl->turnAngle(angle);
    Serial.print(F("Turning "));
    Serial.print(abs(angle));
    Serial.print(F(" degrees "));
    Serial.println(angle > 0 ? F("right") : F("left"));
  }

  void setMovementSpeed(float speed) {
    movementCtrl->setMovementSpeed(speed);
    Serial.print(F("Position movement speed set to "));
    Serial.print(speed);
    Serial.println(F(" RPM"));
  }

  // Helper function to parse three floats from a string
  bool parseThreeFloats(String str, float &f1, float &f2, float &f3) {
    int firstSpace = str.indexOf(' ');
    int secondSpace = str.indexOf(' ', firstSpace + 1);

    if (firstSpace == -1 || secondSpace == -1) {
      return false;
    }

    f1 = str.substring(0, firstSpace).toFloat();
    f2 = str.substring(firstSpace + 1, secondSpace).toFloat();
    f3 = str.substring(secondSpace + 1).toFloat();

    return true;
  }
};

#endif // SERIAL_COMMANDS_H

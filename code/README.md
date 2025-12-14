# N20 Robot Base - Arduino Control Software

A comprehensive motor control system for the N20 robotic base with PID-based speed control and a rich serial command interface.

## Features

- **PID Speed Control**: Closed-loop speed control using quadrature encoder feedback
- **Position-Based Movement**: Move specific distances (cm/m) or turn specific angles (degrees)
- **Serial Interface**: Configure and control the robot via serial commands
- **Modular Design**: Easy to modify pin assignments and parameters
- **Real-time Monitoring**: Check motor speeds, encoder counts, and system status
- **Configurable Parameters**: Update wheel diameter, gear ratio, PID gains, and more

## Hardware Requirements

- Arduino Uno (or compatible)
- L298N Motor Driver
- 2x N20 Motors with Quadrature Encoders
- Power supply for motors (recommend 6-12V depending on your N20 motors)

## Required Libraries

Install via Arduino Library Manager:

1. **Encoder by Paul Stoffregen**
   - Library Manager: Search for "Encoder"
   - Or download from: https://www.pjrc.com/teensy/td_libs_Encoder.html

Note: The Encoder library uses interrupts internally, so you don't need to install EnableInterrupt separately.

## Installation

1. Install the required library (Encoder) via Arduino IDE Library Manager
2. Open `n20_robot_base.ino` in Arduino IDE
3. Verify the pin assignments in `pin_map.h` match your wiring
4. Upload to your Arduino Uno
5. Open Serial Monitor at 115200 baud

## Wiring Diagram

### L298N to Arduino
```
L298N Pin    → Arduino Pin
--------------------------
ENA          → Pin 5 (PWM)
IN1          → Pin 7
IN2          → Pin 8
ENB          → Pin 6 (PWM)
IN3          → Pin 9
IN4          → Pin 10
GND          → GND
```

### Encoders to Arduino
```
Encoder      → Arduino Pin
--------------------------
Left Enc A   → Pin 2 (INT0)
Left Enc B   → Pin 4
Right Enc A  → Pin 3 (INT1)
Right Enc B  → Pin 11
Encoder GND  → GND
Encoder VCC  → 5V
```

### Motors to L298N
```
Left Motor   → OUT1 & OUT2
Right Motor  → OUT3 & OUT4
```

### Power
```
Motor Power+ → 12V (or your motor voltage)
Motor Power- → GND (common with Arduino GND)
Arduino      → USB or VIN (7-12V)
```

## File Structure

```
n20_robot_base/
├── n20_robot_base.ino     # Main sketch
├── pin_map.h              # Pin assignments (easy to modify)
├── config.h               # Robot parameters and PID gains
├── PIDController.h        # PID controller implementation
├── MotorController.h      # Motor control with encoder feedback
├── MovementController.h   # Position-based movement (distance/angle)
└── SerialCommands.h       # Serial command parser and interface
```

## Configuration

### Pin Assignments (`pin_map.h`)

Modify this file to change pin assignments. Default configuration uses:
- Pins 2 & 3 for encoder interrupts (required for Arduino Uno)
- Pins 5 & 6 for PWM motor control
- Pins 7-10 for motor direction control

### Robot Parameters (`config.h`)

Default parameters (modify as needed):
- **Wheel Diameter**: 65mm
- **Wheelbase**: 150mm
- **Encoder PPR**: 480 pulses per revolution
- **Gear Ratio**: 100:1 (typical for N20 motors)
- **PID Gains**: Kp=2.0, Ki=0.5, Kd=0.1

## Serial Commands

Connect via Serial Monitor at **115200 baud**. Commands are case-insensitive.

### Speed-Based Movement Commands

Continuous movement at specified speeds (RPM).

```
forward <speed>     - Move forward at specified speed (RPM)
backward <speed>    - Move backward at specified speed (RPM)
left <speed>        - Turn left (left wheel backward, right forward)
right <speed>       - Turn right (left wheel forward, right backward)
speed <speed>       - Set both motors to same speed (RPM)
stop                - Stop all motors
```

**Examples:**
```
forward 30          # Move forward at 30 RPM
backward 20         # Move backward at 20 RPM
left 25             # Turn left at 25 RPM
right 25            # Turn right at 25 RPM
speed 40            # Set both motors to 40 RPM
stop                # Stop motors
```

### Position-Based Movement Commands

Move specific distances or turn specific angles. The robot automatically stops when the target is reached.

```
go <distance> <unit>  - Move forward/backward (units: cm, m, mm)
                        Use negative distance for backward movement
turn <angle>          - Turn specific angle in degrees
                        Positive = turn right, Negative = turn left
movespeed <speed>     - Set speed for position movements (default: 30 RPM)
```

**Examples:**
```
go 50 cm            # Move forward 50 centimeters
go -30 cm           # Move backward 30 centimeters
go 1.5 m            # Move forward 1.5 meters
go 100 mm           # Move forward 100 millimeters
turn 90             # Turn right 90 degrees
turn -180           # Turn left 180 degrees
turn 45             # Turn right 45 degrees
movespeed 25        # Set position movement speed to 25 RPM
```

**Important Notes:**
- Position-based movements require accurate wheel diameter and wheelbase settings
- Use `wheel <diameter>` and `wheelbase <width>` to configure your robot
- The robot will automatically stop when the target distance/angle is reached
- Use `stop` to interrupt a movement in progress

### PID Configuration

```
pid left <kp> <ki> <kd>    - Set left motor PID gains
pid right <kp> <ki> <kd>   - Set right motor PID gains
```

**Examples:**
```
pid left 2.5 0.8 0.15      # Adjust left motor PID
pid right 2.5 0.8 0.15     # Adjust right motor PID
```

### Robot Configuration

```
wheel <diameter>    - Set wheel diameter in mm
wheelbase <width>   - Set wheelbase width in mm
caster <offset>     - Set caster offset from wheel axis in mm
gear <ratio>        - Set gear ratio
ppr <pulses>        - Set encoder pulses per revolution
turncal <factor>    - Set turning calibration factor (0.8-1.2)
```

**Examples:**
```
wheel 65            # Set wheel diameter to 65mm
wheelbase 150       # Set wheelbase to 150mm
caster 130          # Set caster offset to 130mm
gear 100            # Set gear ratio to 100:1
ppr 480             # Set encoder PPR to 480
turncal 1.05        # Calibrate turning (if robot turns too little)
```

**For accurate positioning, see [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)**

### Status & Diagnostics

```
status              - Print current motor speeds and encoder counts
config              - Print current configuration
reset encoders      - Reset encoder counts to zero
help                - Show all available commands
```

## Usage Examples

### Basic Movement Test
```
# Start serial monitor at 115200 baud
# Type commands one at a time:

forward 20          # Robot moves forward slowly
stop                # Stop
backward 20         # Robot moves backward
stop                # Stop
left 15             # Robot turns left
stop                # Stop
right 15            # Robot turns right
stop                # Stop
```

### Check System Status
```
status              # See current speeds and encoder counts
config              # See all configuration parameters
```

### PID Tuning

1. Start with low speed:
   ```
   forward 20
   ```

2. Monitor performance:
   ```
   status
   ```

3. Adjust PID gains if needed:
   ```
   pid left 3.0 0.6 0.2
   pid right 3.0 0.6 0.2
   ```

4. Test again:
   ```
   forward 30
   status
   ```

### Configure Robot Parameters

If you have different wheels or encoders:
```
wheel 70            # 70mm diameter wheels
gear 150            # 150:1 gear ratio
ppr 360             # 360 PPR encoder
config              # Verify changes
```

### Position-Based Movement Examples

Navigate a square pattern:
```
# First, ensure your robot is configured correctly
config              # Check wheel diameter and wheelbase

# Navigate a 1m x 1m square
go 1 m              # Move forward 1 meter
turn 90             # Turn right 90 degrees
go 1 m              # Move forward 1 meter
turn 90             # Turn right 90 degrees
go 1 m              # Move forward 1 meter
turn 90             # Turn right 90 degrees
go 1 m              # Move forward 1 meter
turn 90             # Turn right 90 degrees (back to start)
```

Test positioning accuracy:
```
# Mark starting position
go 50 cm            # Move forward 50cm
turn 180            # Turn around
go 50 cm            # Move back to start
turn 180            # Face original direction

# Check if robot returned to marked position
# If not accurate, adjust wheel diameter or wheelbase
```

Obstacle avoidance pattern:
```
go 30 cm            # Approach obstacle
turn -90            # Turn left
go 20 cm            # Go around
turn 90             # Turn right
go 40 cm            # Pass obstacle
turn 90             # Turn right
go 20 cm            # Return to path
turn -90            # Turn left to original heading
```

## PID Tuning Guide

Default PID gains may need adjustment based on your specific motors and load:

1. **Start Conservative**: Use low gains (Kp=1.0, Ki=0.1, Kd=0.05)
2. **Increase Kp**: Gradually increase until system responds quickly
3. **Add Ki**: Increase to eliminate steady-state error
4. **Add Kd**: Increase to reduce oscillations
5. **Fine-tune**: Adjust all three for optimal performance

**Signs of poor tuning:**
- **Oscillation**: Reduce Kp or increase Kd
- **Slow response**: Increase Kp
- **Steady-state error**: Increase Ki
- **Overshoot**: Reduce Kp or increase Kd

## Troubleshooting

### Motors don't move
- Check power supply to L298N
- Verify wiring connections
- Check motor power jumpers on L298N
- Test with `forward 50` for higher speed

### Erratic speed readings
- Check encoder wiring
- Verify encoder power (5V)
- Ensure encoder pins are on interrupt-capable pins (2 & 3)
- Check for loose connections

### Motors run at full speed
- PID may be saturating - check target speed
- Verify encoder feedback with `status` command
- Check encoder wiring (A/B might be swapped)

### Serial commands not working
- Verify baud rate is 115200
- Check for line ending settings (NL or CR+NL)
- Type `help` to verify communication

## Advanced Customization

### Adding New Commands

Edit `SerialCommands.h` and add your command in the `processCommand()` function:

```cpp
else if (cmd.startsWith("mycommand ")) {
  // Your code here
}
```

### Changing Control Loop Frequency

Edit `config.h` and modify:
```cpp
unsigned long controlLoopInterval = 50;  // 50ms = 20Hz
```

### Adding Sensors

Add pin definitions to `pin_map.h`:
```cpp
#define ULTRASONIC_TRIG  12
#define ULTRASONIC_ECHO  13
```

## License

Open source - feel free to modify and share!

## Contributing

This is part of the n20-robotic-base project. Contributions welcome!

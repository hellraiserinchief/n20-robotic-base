# Robot Geometry Calibration Guide

This guide helps you measure and configure your robot's physical parameters for accurate position-based movement.

## Understanding Your Robot Configuration

Your robot has a **three-point contact system**:
- Two drive wheels (differential drive)
- One caster wheel
- Circular base

This configuration affects turning accuracy due to:
- Caster wheel drag
- Center of mass distribution
- Contact point geometry
- Wheel slippage

## Required Measurements

### 1. Wheel Diameter

**How to measure:**
1. Place robot wheel on flat surface
2. Mark a point on the wheel
3. Roll wheel exactly one rotation
4. Measure the distance traveled in mm

**Alternative method:**
1. Measure wheel diameter with calipers
2. Calculate: `circumference = π × diameter`
3. Verify by rolling test

**Serial command:**
```
wheel 65.0       # Set wheel diameter to 65.0mm
```

### 2. Wheelbase

Distance between the center of the left and right drive wheels.

**How to measure:**
1. Measure from center of left wheel to center of right wheel
2. Measure perpendicular to the wheel axis
3. Measure in mm

```
     ●---------●
   Left      Right
    |<---W--->|

W = Wheelbase
```

**Serial command:**
```
wheelbase 150.0  # Set wheelbase to 150.0mm
```

### 3. Caster Offset

Distance from the drive wheel axis to the caster wheel.

**How to measure:**

For an equilateral triangle configuration:
```
        Caster
          *
         /|\
        / | \
       /  |  \     ← Measure this perpendicular distance
      /   |   \      (from wheelbase line to caster)
     *----+----*
   Left       Right
```

1. Draw an imaginary line between the two drive wheels
2. Measure perpendicular distance from this line to the caster wheel center
3. **Positive**: caster is in FRONT of drive wheels
4. **Negative**: caster is BEHIND drive wheels

**For equilateral triangle:**
- If wheelbase = W, then caster offset = W × √3 / 2 ≈ W × 0.866

**Example:**
- Wheelbase = 150mm
- Caster offset = 150 × 0.866 = 129.9mm (approximately 130mm)

**Serial command:**
```
caster 130.0     # Set caster offset to 130.0mm
```

### 4. Gear Ratio

Check your N20 motor specifications.

Common N20 gear ratios: 10:1, 20:1, 50:1, 100:1, 150:1, 200:1, 298:1

**Serial command:**
```
gear 100.0       # Set gear ratio to 100:1
```

### 5. Encoder PPR (Pulses Per Revolution)

Check encoder specifications. Common values: 360, 480, 600

**Serial command:**
```
ppr 480          # Set encoder PPR to 480
```

## Turning Calibration Procedure

After setting basic parameters, you need to calibrate turning accuracy.

### Step 1: Initial Test

1. Mark robot's starting position and orientation
2. Command a 360° turn:
   ```
   turn 360
   ```
3. Observe if robot completes exactly one full rotation

### Step 2: Measure Actual Turn

If robot doesn't complete 360°:

**Case A: Robot turns LESS than commanded**
- Example: You command `turn 360` but robot only turns 340°
- Actual turn = 340°, Commanded = 360°
- Calibration factor = 360 / 340 = 1.059

**Case B: Robot turns MORE than commanded**
- Example: You command `turn 360` but robot turns 380°
- Actual turn = 380°, Commanded = 360°
- Calibration factor = 360 / 380 = 0.947

### Step 3: Apply Calibration

```
turncal 1.059    # If robot turns too little
# or
turncal 0.947    # If robot turns too much
```

### Step 4: Verify

1. Reset robot position
2. Test again with `turn 360`
3. Fine-tune if needed
4. Test with different angles: `turn 90`, `turn 180`, `turn -90`

### Step 5: Test Distance Accuracy

1. Mark starting position
2. Command: `go 1 m`
3. Measure actual distance traveled
4. If inaccurate, double-check wheel diameter measurement

## Quick Calibration Commands

```
# Display current configuration
config

# Set all parameters for equilateral triangle robot
wheel 65.0       # Your wheel diameter
wheelbase 150.0  # Distance between wheels
caster 130.0     # For equilateral: wheelbase × 0.866
gear 100.0       # Your motor gear ratio
ppr 480          # Your encoder specification

# Calibrate turning (adjust after testing)
turncal 1.0      # Start with 1.0, then adjust

# Verify settings
config
```

## Systematic Calibration Workflow

### 1. Measure and Set Basic Parameters
```
wheel 65.0
wheelbase 150.0
caster 130.0
gear 100.0
ppr 480
config           # Verify all settings
```

### 2. Test Forward Movement
```
go 50 cm         # Move forward 50cm
# Measure actual distance
# If inaccurate, adjust wheel diameter
```

### 3. Test Turning Accuracy
```
turn 360         # Full rotation
# Observe actual rotation
# Calculate and set turncal factor
```

### 4. Fine-Tune Turning
```
turncal 1.05     # Example adjustment
turn 360         # Test again
turn 90          # Test smaller angle
turn -180        # Test reverse direction
```

### 5. Test Complex Movements
```
# Square pattern test
go 1 m
turn 90
go 1 m
turn 90
go 1 m
turn 90
go 1 m
turn 90
# Robot should return to start position
```

## Factors Affecting Accuracy

### Mechanical Factors
- **Wheel diameter variation**: Worn wheels, tire pressure
- **Wheelbase measurement**: Must be precise
- **Caster drag**: Affects turning
- **Floor surface**: Smooth vs. rough affects slip

### Tuning Factors
- **PID tuning**: Poor PID can cause overshoot/undershoot
- **Speed**: Lower speeds = more accurate
- **Battery voltage**: Low battery affects motor torque

### Environmental Factors
- **Surface friction**: Carpet vs. tile vs. concrete
- **Surface levelness**: Slopes affect straight movement
- **Wheel slippage**: Higher speeds increase slip

## Tips for Best Accuracy

1. **Calibrate on the surface you'll use**: Different surfaces have different friction
2. **Use consistent speed**: Calibrate at the speed you'll operate
3. **Check wheel condition**: Ensure wheels aren't slipping on hubs
4. **Verify encoder connections**: Ensure counts are accurate
5. **Test repeatedly**: Average results from multiple tests
6. **Low battery affects accuracy**: Calibrate with fresh battery

## Troubleshooting

### Robot doesn't go straight
- Check that both motors have similar PID gains
- Verify both wheels have same diameter
- Check for mechanical binding
- Ensure both encoders working correctly

### Turns are inconsistent
- Caster may be dragging unevenly
- Check for debris on wheels or floor
- Verify wheelbase measurement
- Ensure motor mounting is rigid

### Distance is inaccurate
- Verify wheel diameter (most common issue)
- Check for wheel slippage
- Verify encoder PPR setting
- Check gear ratio setting

### Robot veers during movement
- Mismatched PID gains between motors
- Wheels have different diameters
- Mechanical friction or binding
- Encoder issues

## Advanced: Calculating Caster Offset

If your robot configuration is an equilateral triangle:

```
Given:
  - Wheelbase (W) = distance between drive wheels
  - Triangle is equilateral

The caster offset (perpendicular distance):
  Caster Offset = W × sin(60°)
  Caster Offset = W × (√3 / 2)
  Caster Offset ≈ W × 0.866

Example:
  If wheelbase = 150mm
  Caster offset = 150 × 0.866 = 129.9mm
```

## Saving Your Calibration

The robot doesn't save settings to EEPROM. You have two options:

### Option 1: Update config.h
Edit `/code/n20_robot_base/config.h` with your calibrated values:
```cpp
float wheelDiameter = 65.0;        // Your measured value
float wheelBase = 150.0;           // Your measured value
float casterOffset = 130.0;        // Your measured value
float turningCalibration = 1.05;   // Your calibrated value
```

### Option 2: Create startup script
Type commands each time at startup, or add to your control software.

## Example Calibration Session

```
# Connect to serial monitor (115200 baud)

# Step 1: Set known parameters
wheel 65.0
wheelbase 150.0
caster 130.0
gear 100.0
ppr 480
config

# Step 2: Test forward movement
go 50 cm
# Measured: 49.5cm (close enough)

# Step 3: Test turning
turn 360
# Robot turns 380° (too much!)

# Step 4: Calculate calibration
# turncal = 360 / 380 = 0.947

turncal 0.947
config

# Step 5: Verify
turn 360
# Now robot turns 360° (perfect!)

# Step 6: Test smaller angles
turn 90
turn -90
# Both accurate!

# Step 7: Complex test
go 1 m
turn 90
go 1 m
turn 90
go 1 m
turn 90
go 1 m
turn 90
# Robot returns to start!
```

You're now calibrated and ready for accurate position-based navigation!

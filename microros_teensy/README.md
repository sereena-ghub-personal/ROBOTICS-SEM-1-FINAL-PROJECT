# micro-ROS Teensy Motor Controller

This directory contains the micro-ROS firmware for Teensy microcontroller that handles motor control via ROS 2 topics.

## Overview

The firmware subscribes to `/cmd_vel` (geometry_msgs/Twist) messages and controls differential drive motors using PWM signals.

## Hardware Setup

- **Teensy 4.1** (or Teensy 4.0)
- **Pin 5**: Left motor PWM
- **Pin 6**: Right motor PWM
- Motors use servo-style ESC (1000-2000 µs PWM)

## Building and Flashing

### Prerequisites

1. Install PlatformIO:
   ```bash
   pip install platformio
   ```

2. Install micro-ROS agent on your ROS 2 machine:
   ```bash
   sudo apt install ros-humble-micro-ros-agent
   # Or for Iron:
   sudo apt install ros-iron-micro-ros-agent
   ```

### Build and Upload

```bash
cd microros_teensy

# Build
pio run

# Upload to Teensy
pio run --target upload
```

## Running the System

### 1. Start the micro-ROS Agent

The micro-ROS agent bridges serial communication to ROS 2:

```bash
# Find your Teensy serial port (usually /dev/ttyACM0 or /dev/ttyUSB0)
ls /dev/ttyACM* /dev/ttyUSB*

# Start the agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### 2. Start the ROS 2 Nodes

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch the lane following system
ros2 launch lane_following_robot lane_following.launch.py
```

### 3. Start Following

In the track_controller terminal, press `r` to start following the lane.

## ROS 2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

- `linear.x`: Forward velocity (-1.0 to 1.0)
- `angular.z`: Angular velocity (-1.0 to 1.0, positive = turn left)

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/motor_status` | std_msgs/String | Motor status messages |

## Testing

You can test the motor controller manually:

```bash
# Send a forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -1

# Send a stop command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.2}}" -1
```

## Safety Features

- **Command Timeout**: Motors stop automatically if no command received for 500ms
- **Value Clamping**: All velocity values are clamped to [-1.0, 1.0]
- **Neutral State**: Motors default to 1500µs (stopped)

## Motor Configuration

Adjust these constants in `src/main.cpp` based on your motor setup:

```cpp
constexpr int PIN_LEFT = 5;      // Left motor pin
constexpr int PIN_RIGHT = 6;     // Right motor pin
constexpr int SCALE_US = 400;    // PWM scaling factor
constexpr int LEFT_SIGN = -1;    // Left motor direction
constexpr int RIGHT_SIGN = +1;   // Right motor direction
```

## Troubleshooting

### Agent not connecting
- Check that Teensy is properly connected
- Verify the serial port: `ls /dev/ttyACM*`
- Check permissions: `sudo chmod 666 /dev/ttyACM0`
- Try adding yourself to dialout group: `sudo usermod -a -G dialout $USER`

### Motors not responding
- Verify power supply to motors/ESC
- Check PWM signal with oscilloscope
- Test with the STOP command to verify serial communication

### LED Blinking Fast
- Indicates micro-ROS initialization error
- Check agent is running before Teensy boots
- Power cycle Teensy after starting agent

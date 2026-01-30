# Lane-Following Robot

A ROS 2-based autonomous lane-following robot that uses computer vision and adaptive thresholding to detect and follow lane markings in real-time. Now with **micro-ROS Teensy integration** for native ROS 2 communication.

## Overview

This project implements a complete lane-following system with:
- Real-time camera-based lane detection using OpenCV
- Proportional control for smooth steering
- **micro-ROS** motor control via Teensy microcontroller
- Native ROS 2 topic communication (geometry_msgs/Twist)
- Video recording for debugging and analysis

## System Architecture

```
USB CAMERA
    |
    v
CAMERA PUBLISHER (ROS 2 Node)
    |
    | publishes /camera_image
    |
    +---> TRACK CONTROLLER (ROS 2 Node)
    |         |
    |         +---> Lane Detection (OpenCV)
    |         +---> Error Calculation
    |         +---> Proportional Steering Control
    |         +---> publishes /cmd_vel (Twist)
    |                   |
    |                   v
    |              MICRO-ROS AGENT (serial bridge)
    |                   |
    |                   v
    |              TEENSY MICROCONTROLLER (micro-ROS)
    |                   |
    |                   +---> subscribes /cmd_vel
    |                   +---> Motor Mixing (V,W -> L,R)
    |                   +---> PWM Signal Generation
    |                   +---> DC MOTORS
    |
    +---> CAMERA RECORDER (ROS 2 Node)
              |
              +---> Records MP4 files
```

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Teensy 4.1/4.0 | ARM Cortex-M4 with micro-ROS support |
| USB Webcam | 640x480 @ 30 FPS |
| DC Motors (2x) | Differential drive configuration |
| Servo PWM Driver | 1000-2000 microsecond range |
| USB Cable | Serial communication (115200 baud) |

## Software Requirements

- ROS 2 (tested with Humble/Iron)
- Python 3.8+
- OpenCV (`cv2`)
- cv_bridge
- micro-ROS agent (`ros2-humble-micro-ros-agent`)
- PlatformIO (for Teensy firmware)

### Install Dependencies

```bash
# ROS 2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-micro-ros-agent

# Python dependencies
pip install opencv-python numpy pyserial

# PlatformIO (for Teensy development)
pip install platformio
```

## Installation

### 1. Clone and Build the ROS 2 Package

```bash
# Clone the repository
git clone https://github.com/sereena-ghub-personal/ROBOTICS-SEM-1-FINAL-PROJECT.git

# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
ln -s $(pwd)/ROBOTICS-SEM-1-FINAL-PROJECT ~/ros2_ws/src/lane_following_robot

# Build
cd ~/ros2_ws
colcon build --packages-select lane_following_robot
source install/setup.bash
```

### 2. Flash the Teensy with micro-ROS Firmware

```bash
cd ~/ros2_ws/src/lane_following_robot/microros_teensy

# Build and upload
pio run --target upload
```

See [microros_teensy/README.md](microros_teensy/README.md) for detailed instructions.

## Usage

### Quick Start

1. **Start the micro-ROS agent** (connects Teensy to ROS 2):
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   ```

2. **Launch the lane following system**:
   ```bash
   ros2 launch lane_following_robot lane_following.launch.py
   ```

3. **Start following** (in the track_controller terminal):
   - Press `r` to start following
   - Press `s` to stop

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `lane_following.launch.py` | Camera + Track Controller |
| `record.launch.py` | Video recorder only |
| `full_system.launch.py` | All nodes including recorder |

### Manual Testing

```bash
# Test motor control
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -1

# View camera image
ros2 run rqt_image_view rqt_image_view /camera_image

# View debug output
ros2 run rqt_image_view rqt_image_view /track_debug_image

# Monitor motor status
ros2 topic echo /motor_status
```

## Project Structure

```
ROBOTICS-SEM-1-FINAL-PROJECT/
├── CMakeLists.txt              # ROS 2 build configuration
├── package.xml                 # ROS 2 package manifest
├── setup.py                    # Python package setup
├── scripts/
│   ├── track_controller_node.py    # Lane following control
│   ├── camera_publisher_node.py    # Camera capture
│   └── camera_recorder_node.py     # Video recording
├── launch/
│   ├── lane_following.launch.py    # Main launch file
│   ├── record.launch.py            # Recorder only
│   └── full_system.launch.py       # Full system
├── config/
│   └── track_controller.yaml       # Parameter configuration
├── microros_teensy/
│   ├── platformio.ini              # PlatformIO config
│   ├── src/main.cpp                # micro-ROS motor controller
│   └── README.md                   # Teensy setup guide
├── lane_following_robot/           # Python package
│   └── __init__.py
├── resource/                       # ROS 2 package marker
│   └── lane_following_robot
└── README.md
```

## How It Works

### Lane Detection Algorithm

1. **Image Preprocessing:**
   - Convert to grayscale
   - Apply Gaussian blur (5x5 kernel)
   - Adaptive Gaussian threshold (block size: 61, C: -12)

2. **Morphological Operations:**
   - Opening to remove noise
   - Closing (2x) to fill lane gaps

3. **Lane Finding:**
   - Sample row at 60% from top of ROI
   - Cluster white pixels (gap tolerance: 5px)
   - Support single-line and dual-line detection

4. **Control Output:**
   - Calculate error from lane center
   - Apply proportional control with deadband
   - Rate-limit velocity changes for smooth motion

### Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp` | 0.0008 | Proportional gain |
| `base_v` | 0.30 | Base forward velocity (0-1) |
| `max_w` | 0.50 | Maximum angular velocity (0-1) |
| `deadband_px` | 25.0 | Steering deadband (pixels) |
| `err_alpha` | 0.10 | Error smoothing factor |

### Safety Features

- **Command timeout:** Motors stop after 500ms without commands (micro-ROS)
- **Acceleration limits:** Smooth velocity transitions
- **Lost lane hysteresis:** Continues last steering for 40 frames
- **Value clamping:** All velocities clamped to [-1.0, 1.0]

## ROS 2 Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/camera_image` | `sensor_msgs/Image` | camera_publisher | Raw camera ROI (BGR8) |
| `/camera_image_debug` | `sensor_msgs/Image` | camera_publisher | Debug visualization |
| `/track_debug_image` | `sensor_msgs/Image` | track_controller | Lane detection overlay |
| `/cmd_vel` | `geometry_msgs/Twist` | track_controller | Velocity commands |
| `/motor_status` | `std_msgs/String` | teensy | Motor status |

## Recordings

Video recordings are saved to:
```
~/ros2_ws/src/final_project/recordings/
├── camera_raw_[timestamp].mp4
└── camera_debug_[timestamp].mp4
```

## Troubleshooting

**micro-ROS agent not connecting:**
- Verify Teensy serial port: `ls /dev/ttyACM*`
- Check permissions: `sudo chmod 666 /dev/ttyACM0`
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Power cycle Teensy after starting agent

**Camera not detected:**
- Check USB connection
- Verify device ID (default: 0)
- Test with: `v4l2-ctl --list-devices`

**Motors not responding:**
- Verify micro-ROS agent is running and connected
- Check `/motor_status` topic for feedback
- Test with manual cmd_vel publish

**Poor lane detection:**
- Adjust `adapt_block_size` and `adapt_C` parameters
- Ensure adequate lighting
- Verify camera ROI captures the track

## Legacy Support

The original serial-based files are preserved for reference:
- `track controller` - Original serial-based controller
- `camera publisher` - Original camera node
- `camera record` - Original recorder
- `Teensy code for motors` - Original Arduino firmware

## License

This project was developed as part of a Robotics Semester 1 Final Project.

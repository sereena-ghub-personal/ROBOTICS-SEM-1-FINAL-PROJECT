# Lane-Following Robot

A ROS 2-based autonomous lane-following robot that uses computer vision and adaptive thresholding to detect and follow lane markings in real-time.

## Overview

This project implements a complete lane-following system with:
- Real-time camera-based lane detection using OpenCV
- Proportional control for smooth steering
- Differential drive motor control via Teensy microcontroller
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
    |         +---> Serial Command: "VW v w"
    |                   |
    |                   v
    |              TEENSY MICROCONTROLLER
    |                   |
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
| Teensy Microcontroller | ARM Cortex-M4 for motor control |
| USB Webcam | 640x480 @ 30 FPS |
| DC Motors (2x) | Differential drive configuration |
| Servo PWM Driver | 1000-2000 microsecond range |
| USB Cable | Serial communication (9600 baud) |

## Software Requirements

- ROS 2 (tested with Humble/Iron)
- Python 3.8+
- OpenCV (`cv2`)
- cv_bridge
- Arduino IDE or PlatformIO (for Teensy firmware)

### Python Dependencies

```bash
pip install opencv-python numpy
```

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/sereena-ghub-personal/ROBOTICS-SEM-1-FINAL-PROJECT.git
   cd ROBOTICS-SEM-1-FINAL-PROJECT
   ```

2. **Set up ROS 2 workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src/final_project
   cp "track controller" ~/ros2_ws/src/final_project/track_controller.py
   cp "camera publisher" ~/ros2_ws/src/final_project/camera_publisher.py
   cp "camera record" ~/ros2_ws/src/final_project/camera_recorder.py
   ```

3. **Upload Teensy firmware:**
   - Open `Teensy code for motors` in Arduino IDE
   - Select Teensy board
   - Upload to microcontroller

## Usage

### Running the Robot

1. **Start the camera publisher:**
   ```bash
   python3 camera_publisher.py
   ```

2. **Start the track controller:**
   ```bash
   python3 track_controller.py
   ```

3. **Optional - Record video:**
   ```bash
   python3 camera_recorder.py
   ```

### Teensy Serial Commands

| Command | Description |
|---------|-------------|
| `VW v w` | Set linear velocity (v) and angular velocity (w) |
| `STOP` | Emergency stop |
| `TEST` | Run self-test sequence |

## Project Structure

```
ROBOTICS-SEM-1-FINAL-PROJECT/
├── track controller      # Lane detection and control logic (Python/ROS 2)
├── camera publisher      # Camera capture and ROS image publishing
├── camera record         # Video recording node
├── Teensy code for motors # Motor control firmware (C++/Arduino)
└── README.md
```

## How It Works

### Lane Detection Algorithm

1. **Image Preprocessing:**
   - Convert to grayscale
   - Apply Gaussian blur (5x5 kernel)
   - Adaptive Gaussian threshold (block size: 51, C: -8)

2. **Morphological Operations:**
   - Opening to remove noise
   - Closing (2x) to fill lane gaps

3. **Lane Finding:**
   - Sample middle row of ROI
   - Cluster white pixels (gap tolerance: 5px)
   - Detect single or dual lane mode

4. **Control Output:**
   - Calculate error from lane center
   - Apply proportional control with deadband
   - Rate-limit velocity changes for smooth motion

### Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kp` | 0.0012 | Proportional gain |
| `base_v` | 0.32 | Base forward velocity |
| `max_w` | 0.80 | Maximum angular velocity |
| `deadband_px` | 40.0 | Steering deadband (pixels) |
| `err_alpha` | 0.15 | Error smoothing factor |

### Safety Features

- **Command timeout:** Auto-stop after 1200ms without commands
- **Acceleration limits:** Smooth velocity transitions
- **Lost lane hysteresis:** Continues last steering for 40 frames
- **Value clamping:** Prevents motor overdrive

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera_image` | `sensor_msgs/Image` | Raw camera ROI (BGR8) |
| `/camera_image_debug` | `sensor_msgs/Image` | Debug visualization |
| `/track_debug_image` | `sensor_msgs/Image` | Lane detection overlay |

## Recordings

Video recordings are saved to:
```
~/ros2_ws/src/final_project/recordings/
├── camera_raw_[timestamp].mp4
└── camera_debug_[timestamp].mp4
```

## Troubleshooting

**Camera not detected:**
- Check USB connection
- Verify device ID in `camera_publisher.py` (default: 0)

**Motors not responding:**
- Verify Teensy serial connection
- Check baud rate (9600)
- Run `TEST` command for diagnostics

**Poor lane detection:**
- Adjust `adapt_block_size` and `adapt_c` parameters
- Ensure adequate lighting
- Verify camera ROI captures the track

## License

This project was developed as part of a Robotics Semester 1 Final Project.

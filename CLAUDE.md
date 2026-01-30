# CLAUDE.md - AI Assistant Guide for Lane-Following Robot Project

## Project Overview

This is a **lane-following robot system** built for a Robotics Semester 1 Final Project. The system uses computer vision with ROS 2 to detect lane markings and control a differential-drive robot via a Teensy microcontroller.

**Key Technologies:**
- ROS 2 (Python) for distributed node-based architecture
- OpenCV for computer vision and image processing
- Arduino/Teensy for embedded motor control
- Serial communication between host and microcontroller

## Codebase Structure

```
/home/user/ROBOTICS-SEM-1-FINAL-PROJECT/
├── CLAUDE.md                    # This file
├── Teensy code for motors       # Arduino/C++ firmware for Teensy
├── camera publisher             # ROS 2 node: camera capture and ROI publishing
├── camera record                # ROS 2 node: video recording from ROS topics
└── track controller             # ROS 2 node: lane detection and motor control
```

**Note:** Source files do not have extensions - they are standalone scripts/code.

## Component Architecture

### Data Flow

```
USB Camera → CameraPublisher → /camera_image → TrackController → Serial → Teensy → Motors
                                                      ↓
                                              /track_debug_image
                                                      ↓
                                            CameraROSRecorder → MP4 files
```

### 1. Teensy Motor Control (`Teensy code for motors`)

**Language:** Arduino C++
**Purpose:** Differential drive motor control via PWM

**Key Constants:**
- `PIN_LEFT = 5`, `PIN_RIGHT = 6` - Servo motor pins
- `NEUTRAL_US = 1500` - Neutral PWM (motors stopped)
- `SCALE_US = 400` - PWM scale factor for velocity
- `CMD_TIMEOUT_MS = 1200` - Safety timeout (auto-stop)
- `LEFT_SIGN = -1`, `RIGHT_SIGN = +1` - Direction signs

**Serial Protocol (9600 baud):**
```
VW v w    - Differential drive (v=linear [-1,1], w=angular [-1,1])
STOP      - Emergency stop
TEST      - Self-test routine
```

**Response Format:**
```
OK VW v=0.320 w=0.000 | l=0.320 r=0.320 | l_us=1628 (L:FWD) r_us=1628 (R:FWD)
```

### 2. Camera Publisher (`camera publisher`)

**Language:** Python 3 (ROS 2)
**Purpose:** Capture camera frames and publish ROI

**ROS Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `0` | Camera device index |
| `topic` | `camera_image` | Output topic |
| `debug_topic` | `camera_image_debug` | Debug visualization topic |
| `fps` | `30.0` | Frame rate |
| `width` | `640` | Frame width |
| `height` | `480` | Frame height |
| `roi_height_ratio` | `0.55` | Bottom portion of frame (ROI) |

**Published Topics:**
- `/camera_image` - BGR8 encoded ROI image
- `/camera_image_debug` - Debug overlay with ROI indicator

### 3. Camera Recorder (`camera record`)

**Language:** Python 3 (ROS 2)
**Purpose:** Record video streams to MP4 files

**Subscribed Topics:**
- `camera_image` - Raw camera ROI
- `track_debug_image` - Lane detection debug visualization

**Output Location:** `~/ros2_ws/src/final_project/recordings/`
**Output Format:** `camera_raw_YYYYMMDD_HHMMSS.mp4`, `camera_debug_YYYYMMDD_HHMMSS.mp4`

### 4. Track Controller (`track controller`)

**Language:** Python 3 (ROS 2)
**Purpose:** Lane detection and robot motion control

**Vision Pipeline:**
1. Convert to grayscale and blur
2. Adaptive thresholding (Gaussian, block size 51)
3. Morphological filtering (open + close)
4. Lane detection via white pixel clustering
5. Single-line or dual-line mode support

**Key ROS Parameters:**

| Category | Parameter | Default | Description |
|----------|-----------|---------|-------------|
| **Serial** | `port` | `/dev/ttyACM0` | Teensy serial port |
| | `baud` | `9600` | Baud rate |
| **Detection** | `min_white_pixels` | `35` | Min pixels to detect lane |
| | `min_lane_width_px` | `100` | Min valid lane width |
| | `max_lane_width_ratio` | `0.95` | Max lane width as ratio of image |
| | `max_white_ratio` | `0.75` | Max white pixel ratio (reject overexposure) |
| **Threshold** | `adapt_block_size` | `51` | Adaptive threshold block size (odd) |
| | `adapt_C` | `-8` | Adaptive threshold constant |
| **Control** | `kp` | `0.0012` | Proportional gain |
| | `base_v` | `0.32` | Base linear velocity |
| | `max_w` | `0.80` | Max angular velocity |
| | `deadband_px` | `40.0` | Error deadband (straight line) |
| **Smoothing** | `err_alpha` | `0.15` | Error smoothing factor |
| **Rate Limit** | `v_accel_limit` | `0.15` | Linear acceleration limit |
| | `w_accel_limit` | `0.05` | Angular acceleration limit |
| **Single Line** | `assumed_lane_width_px` | `120` | Assumed lane width for single-line mode |

**Keyboard Commands:**
- `r` - START following
- `s` - STOP

**Operating Modes:**
- `IDLE` - Waiting for start command
- `FOLLOW` - Active lane following

## Development Conventions

### Code Style

1. **Python ROS 2 Nodes:**
   - Inherit from `rclpy.node.Node`
   - Use `declare_parameter()` / `get_parameter()` for configuration
   - Clean shutdown with `destroy_node()` and resource cleanup
   - Use CvBridge for ROS Image ↔ OpenCV conversion

2. **Arduino/Teensy Code:**
   - Use `constexpr` for compile-time constants
   - Implement safety timeouts (watchdog pattern)
   - Use `clamp` functions for value bounds checking
   - Detailed serial logging for debugging

3. **Naming Conventions:**
   - Variables: `snake_case`
   - Constants: `UPPER_SNAKE_CASE` (Arduino) or inline values (Python)
   - Node names: `snake_case` (e.g., `track_controller`)
   - Topics: `snake_case` (e.g., `camera_image`, `track_debug_image`)

### Safety Patterns

1. **Watchdog Timeout:** Teensy auto-stops motors after 1200ms without commands
2. **Lost Hysteresis:** Controller waits 40 frames before full stop when lane lost
3. **Rate Limiting:** Smooth acceleration to prevent jerky movements
4. **Graceful Degradation:** 30% velocity reduction while tracking is lost

### Error Handling

1. Serial port errors wrapped in try-catch with logging
2. Resource cleanup in `destroy_node()` methods
3. Graceful KeyboardInterrupt handling

## Common Development Tasks

### Adding a New ROS Parameter

```python
# In __init__():
self.declare_parameter('new_param', default_value)
self.new_param = type(self.get_parameter('new_param').value)
```

### Modifying Vision Pipeline

Edit `_detect_lane()` in `track controller`:
- Thresholding: Lines 198-210
- Morphology: Lines 212-216
- Lane clustering: Lines 234-285

### Adjusting Motor Response

- **Speed:** Modify `base_v` parameter or `SCALE_US` in Teensy code
- **Turning:** Adjust `kp` and `max_w` parameters
- **Smoothness:** Tune `err_alpha`, `v_accel_limit`, `w_accel_limit`

### Testing Motor Control

Send commands directly via serial:
```bash
# Test forward motion
echo "VW 0.3 0.0" > /dev/ttyACM0

# Test rotation
echo "VW 0.0 0.3" > /dev/ttyACM0

# Stop
echo "STOP" > /dev/ttyACM0

# Self-test
echo "TEST" > /dev/ttyACM0
```

## Dependencies

### Python
- `rclpy` - ROS 2 Python client
- `cv_bridge` - ROS Image ↔ OpenCV conversion
- `opencv-python` (`cv2`) - Computer vision
- `numpy` - Numerical operations
- `pyserial` - Serial communication

### Arduino/Teensy
- `Arduino.h` - Arduino core
- `Servo.h` - PWM servo control

## ROS 2 Workspace Integration

This project is designed to run within a ROS 2 workspace at `~/ros2_ws/src/final_project/`.

**Expected Launch Sequence:**
1. Start `camera publisher` node
2. Start `track controller` node
3. (Optional) Start `camera record` node for recording
4. Press `r` in track controller terminal to begin following

## Hardware Configuration

- **Camera:** USB camera at `/dev/video0` (640x480 @ 30fps)
- **Microcontroller:** Teensy at `/dev/ttyACM0` (9600 baud)
- **Motors:** Two servo motors on pins 5 (left) and 6 (right)
- **Drive Type:** Differential drive with PWM control (1000-2000 μs range)

## Debugging

### Debug Visualization

The track controller publishes debug images to `/track_debug_image` showing:
- Binary threshold output
- Detected lane markers (green circles)
- Lane center (red circle)
- Image center line (white)
- Status text overlay

### Common Issues

1. **Robot doesn't move:** Check serial connection, verify Teensy is receiving commands
2. **Erratic turning:** Tune `kp`, increase `deadband_px`, or adjust `err_alpha`
3. **Lane not detected:** Adjust `adapt_block_size`, `adapt_C`, or `min_white_pixels`
4. **Overexposure:** Lower `max_white_ratio` threshold

## Git Workflow

The project uses simple, descriptive commit messages:
- "Add [Component] for [purpose]"
- Example: "Add Teensy code for motor control"

## File Modification Guidelines

When editing this codebase:

1. **Preserve parameter patterns** - Use ROS 2 parameter system for configuration
2. **Maintain safety mechanisms** - Don't remove timeouts or rate limiting
3. **Keep debug visualization** - Essential for tuning and troubleshooting
4. **Test incrementally** - Use Teensy TEST command and manual keyboard controls
5. **Document parameter changes** - Update this file when adding new parameters

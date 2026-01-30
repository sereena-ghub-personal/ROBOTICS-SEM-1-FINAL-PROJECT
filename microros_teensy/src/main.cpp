/**
 * micro-ROS Motor Controller for Teensy
 *
 * Subscribes to /cmd_vel (geometry_msgs/Twist) and controls differential drive motors.
 * Publishes motor status to /motor_status (std_msgs/String).
 *
 * Hardware:
 *   - PIN_LEFT (5): Left motor PWM
 *   - PIN_RIGHT (6): Right motor PWM
 *   - Motors use servo-style PWM (1000-2000 µs)
 */

#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// ==================== HARDWARE CONFIGURATION ====================
constexpr int PIN_LEFT = 5;
constexpr int PIN_RIGHT = 6;
constexpr int PIN_LED = LED_BUILTIN;

// Motor PWM settings (servo-style ESC)
constexpr int NEUTRAL_US = 1500;  // Stop position
constexpr int MIN_US = 1000;      // Full reverse
constexpr int MAX_US = 2000;      // Full forward
constexpr int SCALE_US = 400;     // Scaling factor for velocity

// Direction signs (adjust based on motor wiring)
constexpr int LEFT_SIGN = -1;     // Left forward = < 1500
constexpr int RIGHT_SIGN = +1;    // Right forward = > 1500

// Safety
constexpr unsigned long CMD_TIMEOUT_MS = 500;  // Stop if no command received
constexpr int STOP_BAND_US = 10;  // Deadband around neutral

// ==================== MICRO-ROS CONFIGURATION ====================
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t status_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__String status_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==================== MOTOR OBJECTS ====================
Servo leftMotor;
Servo rightMotor;

// ==================== STATE ====================
unsigned long last_cmd_time = 0;
float current_v = 0.0f;
float current_w = 0.0f;
char status_buffer[128];

// ==================== ERROR HANDLING ====================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while(1) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(100);
    }
}

// ==================== MOTOR FUNCTIONS ====================
static int clamp_us(int us) {
    if (us < MIN_US) return MIN_US;
    if (us > MAX_US) return MAX_US;
    return us;
}

static void setMotorsUs(int l_us, int r_us) {
    leftMotor.writeMicroseconds(clamp_us(l_us));
    rightMotor.writeMicroseconds(clamp_us(r_us));
}

static void stopMotors() {
    setMotorsUs(NEUTRAL_US, NEUTRAL_US);
    current_v = 0.0f;
    current_w = 0.0f;
}

const char* getDirection(int us, int sign) {
    int d = us - NEUTRAL_US;
    if (abs(d) <= STOP_BAND_US) return "STOP";
    bool forward = (sign * d) > 0;
    return forward ? "FWD" : "REV";
}

void applyVelocity(float v, float w) {
    // Clamp inputs
    v = constrain(v, -1.0f, 1.0f);
    w = constrain(w, -1.0f, 1.0f);

    // Differential drive mixing
    // v = linear velocity (forward/backward)
    // w = angular velocity (positive = turn left, negative = turn right)
    float left = v + w;
    float right = v - w;

    // Clamp mixed values
    left = constrain(left, -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    // Convert to PWM microseconds
    int l_us = NEUTRAL_US + LEFT_SIGN * (int)(left * SCALE_US);
    int r_us = NEUTRAL_US + RIGHT_SIGN * (int)(right * SCALE_US);

    setMotorsUs(l_us, r_us);

    current_v = v;
    current_w = w;

    // Update status message
    snprintf(status_buffer, sizeof(status_buffer),
             "v=%.2f w=%.2f L=%d(%s) R=%d(%s)",
             v, w, l_us, getDirection(l_us, LEFT_SIGN),
             r_us, getDirection(r_us, RIGHT_SIGN));
}

// ==================== ROS CALLBACKS ====================
void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;

    // Extract linear.x and angular.z from Twist message
    // linear.x = forward velocity (-1 to 1)
    // angular.z = rotation velocity (-1 to 1, positive = turn left)
    float v = (float)msg->linear.x;
    float w = (float)msg->angular.z;

    applyVelocity(v, w);
    last_cmd_time = millis();

    // Blink LED on command receive
    digitalWrite(PIN_LED, HIGH);
}

// ==================== SETUP ====================
void setup() {
    // Initialize pins
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // LED on during setup

    // Initialize motors
    leftMotor.attach(PIN_LEFT);
    rightMotor.attach(PIN_RIGHT);
    stopMotors();

    // Initialize serial for micro-ROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);  // Wait for micro-ROS agent

    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();

    // Create init options and support
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "teensy_motor_controller", "", &support));

    // Create subscriber for cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Create publisher for motor status
    RCCHECK(rclc_publisher_init_default(
        &status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "motor_status"));

    // Initialize status message
    status_msg.data.data = status_buffer;
    status_msg.data.size = 0;
    status_msg.data.capacity = sizeof(status_buffer);

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
            &cmd_vel_callback, ON_NEW_DATA));

    digitalWrite(PIN_LED, LOW);  // LED off when ready

    snprintf(status_buffer, sizeof(status_buffer), "Teensy motor controller ready");
    status_msg.data.size = strlen(status_buffer);
    RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
}

// ==================== MAIN LOOP ====================
void loop() {
    // Process incoming messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    // Safety timeout - stop motors if no command received
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
        if (current_v != 0.0f || current_w != 0.0f) {
            stopMotors();
            snprintf(status_buffer, sizeof(status_buffer), "TIMEOUT - motors stopped");
            status_msg.data.size = strlen(status_buffer);
            RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
        }
        digitalWrite(PIN_LED, LOW);
    }

    // Publish status periodically
    static unsigned long last_status_time = 0;
    if (millis() - last_status_time > 100) {  // 10 Hz status update
        status_msg.data.size = strlen(status_buffer);
        RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
        last_status_time = millis();
    }
}

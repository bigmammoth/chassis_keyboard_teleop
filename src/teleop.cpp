/**
 * @file teleop.cpp
 * @brief Implementation of a ROS2 node for keyboard-based teleoperation of a mobile robot chassis.
 *
 * This node reads arrow-key events from a Linux evdev input device and maps them to linear
 * and angular velocity targets. Velocities are smoothly ramped using configurable acceleration
 * and deceleration limits, then published as geometry_msgs::msg::Twist on the "cmd_vel" topic.
 * The node also subscribes to wheel odometry on "left_wheel_odom" and logs the robot pose at 1 Hz.
 */

#include "chassis_keyboard_teleop/teleop.hpp"
#include <linux/input.h>
#include <chrono>
#include <fcntl.h>
#include <stdexcept>

/// @namespace teleop
/// @brief Contains classes and functions for keyboard teleoperation.
using namespace std::chrono_literals;
namespace tp = teleop;

/**
 * @class teleop::KeyboardTeleop
 * @brief ROS2 node that converts keyboard input into velocity commands.
 *
 * On construction, the node:
 *   - Declares parameters for maximum speeds, accelerations, and the input device path.
 *   - Creates a publisher for cmd_vel and a subscriber for odometry.
 *   - Opens the specified /dev/input/event* device using libevdev.
 *   - Launches a background thread to read keyboard events.
 *   - Starts a 20 ms wall‐timer to update velocities and publish Twist messages.
 *
 * On destruction, the node:
 *   - Signals the input thread to stop and joins it.
 *   - Frees the libevdev device.
 */

/**
 * @brief Construct a new KeyboardTeleop node.
 *
 * Declares all ROS parameters, initializes publishers and subscribers, opens
 * the evdev device for nonblocking reads, and starts the input handling thread
 * and periodic publish timer.
 *
 * @throws std::runtime_error if opening the input device or initializing libevdev fails.
 */
tp::KeyboardTeleop::KeyboardTeleop()
    : Node("chassis_keyboard_teleop")
{
    /* ---- 读取参数 ---- */
    max_lin_ = declare_parameter("max_linear", 0.8);                     // m/s
    max_ang_ = declare_parameter("max_angular", 1.8);                    // rad/s
    accel_lin_ = declare_parameter("linear_accel", 0.3);                 // m/s²
    decel_lin_ = declare_parameter("linear_decel", 0.4);                 // m/s²
    accel_ang_ = declare_parameter("angular_accel", 1.0);                // rad/s²
    decel_ang_ = declare_parameter("angular_decel", 1.2);                // rad/s²
    event_path_ = declare_parameter("event_path", "/dev/input/event0");  // 键盘设备

    /* ---- ROS pub / sub ---- */
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "left_wheel_odom", 10,
        std::bind(&KeyboardTeleop::odomCallback, this, std::placeholders::_1));

    /* ---- 打开输入设备 ---- */
    int fd = open(event_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
        throw std::runtime_error("Cannot open " + event_path_);
    if (libevdev_new_from_fd(fd, &dev_) < 0)
        throw std::runtime_error("libevdev init failed");

    running_ = true;
    input_thread_ = std::thread(&KeyboardTeleop::inputLoop, this);

    /* ---- 发布定时器 ---- */
    timer_ = create_wall_timer(20ms, std::bind(&KeyboardTeleop::updateAndPublish, this));

    RCLCPP_INFO(get_logger(), "Keyboard teleop started on %s", event_path_.c_str());
}

/**
 * @brief Destroy the KeyboardTeleop node.
 *
 * Stops the input thread if running, waits for it to join, and cleans up the evdev device.
 */
tp::KeyboardTeleop::~KeyboardTeleop()
{
    running_ = false;
    if (input_thread_.joinable())
        input_thread_.join();
    if (dev_)
        libevdev_free(dev_);
}

/**
 * @brief Main loop running in a separate thread to read keyboard events.
 *
 * Continuously calls libevdev_next_event() in nonblocking mode. On receiving
 * EV_KEY events, forwards the key code and value to handleKey(). Sleeps briefly
 * when no event is available to reduce CPU usage.
 */
void tp::KeyboardTeleop::inputLoop()
{
    input_event ev;
    while (running_)
    {
        int rc = libevdev_next_event(dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == LIBEVDEV_READ_STATUS_SUCCESS && ev.type == EV_KEY)
        {
            handleKey(ev.code, ev.value);
        }
        else if (rc == -EAGAIN)
        {
            std::this_thread::sleep_for(5ms);
        }
    }
}

/**
 * @brief Handle a single key press or release event.
 *
 * Updates internal flags for arrow keys based on the provided code and value.
 *
 * @param code Linux input key code (e.g., KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT).
 * @param value Event value (0 = key released, 1 = key pressed).
 */
void tp::KeyboardTeleop::handleKey(int code, int value)
{
    const bool pressed = (value != 0);
    switch (code)
    {
    case KEY_UP:
        key_up_ = pressed;
        break;
    case KEY_DOWN:
        key_down_ = pressed;
        break;
    case KEY_LEFT:
        key_left_ = pressed;
        break;
    case KEY_RIGHT:
        key_right_ = pressed;
        break;
    default:
        break;
    }
}

/**
 * @brief Smoothly approach a target value from the current value.
 *
 * Increases or decreases the current value by at most step, clamped to the range
 * [-limit, +limit], until it reaches the target.
 *
 * @param target Desired end value.
 * @param current Present value before adjustment.
 * @param step Maximum change to apply in this update.
 * @param limit Maximum absolute magnitude allowed.
 * @return double New adjusted value clamped within [-limit, limit].
 */
static double approach(double target, double current, double step, double limit)
{
    if (current < target)
    {
        current = std::min(current + step, target);
    }
    else if (current > target)
    {
        current = std::max(current - step, target);
    }
    return std::clamp(current, -limit, limit);
}

/**
 * @brief Compute and publish the current velocity command.
 *
 * Determines linear and angular targets based on arrow-key flags,
 * applies acceleration or deceleration limits via approach(),
 * constructs a geometry_msgs::msg::Twist, and publishes it on the cmd_vel topic.
 * Runs at 50 Hz (20 ms period).
 */
void tp::KeyboardTeleop::updateAndPublish()
{
    constexpr double dt = 0.02; // 20 ms
    /* 目标速度 */
    double tgt_lin = 0.0, tgt_ang = 0.0;
    if (key_up_)
        tgt_lin += max_lin_;
    if (key_down_)
        tgt_lin -= max_lin_;
    if (key_left_)
        tgt_ang += max_ang_;
    if (key_right_)
        tgt_ang -= max_ang_;

    /* 加减速 */
    vel_lin_ = approach(tgt_lin, vel_lin_, (tgt_lin ? accel_lin_ : decel_lin_) * dt, max_lin_);
    vel_ang_ = approach(tgt_ang, vel_ang_, (tgt_ang ? accel_ang_ : decel_ang_) * dt, max_ang_);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = vel_lin_;
    msg.angular.z = vel_ang_;
    cmd_pub_->publish(msg);
}

/**
 * @brief Odometry callback that logs the robot’s pose at a throttled rate.
 *
 * Receives nav_msgs::msg::Odometry messages from the "left_wheel_odom" topic
 * and logs x, y, and approximate theta (orientation.z) at most once per second.
 *
 * @param msg Shared pointer to the incoming Odometry message.
 */
void tp::KeyboardTeleop::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Odom x=%.2f y=%.2f theta≈%.2f",
                         msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.orientation.z);
}

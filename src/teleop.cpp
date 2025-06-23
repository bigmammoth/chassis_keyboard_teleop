#include "chassis_keyboard_teleop/teleop.hpp"
#include <linux/input.h>
#include <chrono>
#include <fcntl.h>
#include <stdexcept>

using namespace std::chrono_literals;
namespace tp = teleop;

/* ------------ 构造 / 析构 ------------ */
tp::KeyboardTeleop::KeyboardTeleop()
    : Node("chassis_keyboard_teleop")
{
    /* ---- 读取参数 ---- */
    max_lin_ = declare_parameter("max_linear", 0.8);                    // m/s
    max_ang_ = declare_parameter("max_angular", 1.8);                   // rad/s
    accel_lin_ = declare_parameter("linear_accel", 0.3);                // m/s²
    decel_lin_ = declare_parameter("linear_decel", 0.4);                // m/s²
    accel_ang_ = declare_parameter("angular_accel", 1.0);               // rad/s²
    decel_ang_ = declare_parameter("angular_decel", 1.2);               // rad/s²
    event_path_ = declare_parameter("event_path", "/dev/input/event0"); // 键盘设备

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

tp::KeyboardTeleop::~KeyboardTeleop()
{
    running_ = false;
    if (input_thread_.joinable())
        input_thread_.join();
    if (dev_)
        libevdev_free(dev_);
}

/* ------------ 输入线程 ------------ */
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

/* ------------ 速度积分与发布 ------------ */
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

/* ------------ 里程计回调 ------------ */
void tp::KeyboardTeleop::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Odom x=%.2f y=%.2f theta≈%.2f",
                         msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.orientation.z);
}

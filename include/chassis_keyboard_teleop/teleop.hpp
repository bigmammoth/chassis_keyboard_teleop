#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <libevdev/libevdev.h> // 监听 /dev/input/eventX
#include <thread>
#include <atomic>
#include <unordered_map>

namespace teleop
{
    class KeyboardTeleop : public rclcpp::Node
    {
    public:
        KeyboardTeleop();
        ~KeyboardTeleop() override;

    private:
        /* ---------- 输入线程 ---------- */
        void inputLoop();
        void handleKey(int code, int value); // value: 0=release, 1=press, 2=hold
        std::thread input_thread_;
        std::atomic<bool> running_{false};
        struct libevdev *dev_{nullptr};

        /* ---------- 加/减速逻辑 ---------- */
        void updateAndPublish();
        rclcpp::TimerBase::SharedPtr timer_;
        double vel_lin_{0.0}, vel_ang_{0.0};
        bool key_up_{false}, key_down_{false}, key_left_{false}, key_right_{false};

        /* ---------- ROS 通信 ---------- */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /* ---------- 参数 ---------- */
        double max_lin_, max_ang_;
        double accel_lin_, accel_ang_;
        double decel_lin_, decel_ang_;
        std::string event_path_;
    };
} // namespace teleop

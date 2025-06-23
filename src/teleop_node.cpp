#include "rclcpp/rclcpp.hpp"
#include "chassis_keyboard_teleop/teleop.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<teleop::KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

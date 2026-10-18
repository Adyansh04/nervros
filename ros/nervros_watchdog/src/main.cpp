#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nervros_watchdog/watchdog.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nervros_watchdog::Watchdog>());
    rclcpp::shutdown();
    return 0;
}

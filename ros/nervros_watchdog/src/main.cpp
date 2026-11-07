#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nervros_watchdog/watchdog.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // A bad parameter throws from the constructor: said, and non-zero, so the launch restarts it.
    int code = 0;
    try
    {
        rclcpp::spin(std::make_shared<nervros_watchdog::Watchdog>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("nervros_watchdog"), "%s", e.what());
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}

#pragma once

#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>
#include <map>
#include <nervros_interfaces/msg/open_goals.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace nervros_watchdog
{

/**
 * @brief Cancels the action goals an agent's own motions have open once the agent goes quiet.
 *
 * The agent sends its open goals again and again while any is open. When none has arrived for
 * timeout_s while some were open, each is cancelled by its id and MoveIt is told to stop: a walk
 * or an arm move the agent started does not run on after it has gone.
 */
class Watchdog : public rclcpp::Node
{
public:
    explicit Watchdog(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void onOpenGoals(const nervros_interfaces::msg::OpenGoals& goals);
    void check();
    void cancelAll();

    std::chrono::nanoseconds                                                       timeout_;
    rclcpp::Subscription<nervros_interfaces::msg::OpenGoals>::SharedPtr            open_goals_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            moveit_stop_;
    rclcpp::TimerBase::SharedPtr                                                   timer_;
    std::map<std::string, rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> cancellers_;
    nervros_interfaces::msg::OpenGoals                                             open_;
    std::chrono::steady_clock::time_point                                          heard_;
};

}  // namespace nervros_watchdog

#pragma once

#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>
#include <cstdint>
#include <map>
#include <nervros_interfaces/msg/open_goals.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

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
    /// A goal to cancel, until when its server may still come up to be asked.
    struct Pending
    {
        std::string                           action;
        unique_identifier_msgs::msg::UUID     goal_id;
        std::chrono::steady_clock::time_point until;
    };

    /// A cancel sent and not answered yet, and by when its answer must come.
    struct Asked
    {
        Pending                               goal;
        std::int64_t                          request_id = 0;
        std::chrono::steady_clock::time_point by;
    };

    /// What one agent last said it has open, and when.
    struct Agent
    {
        nervros_interfaces::msg::OpenGoals    open;
        std::chrono::steady_clock::time_point heard;
    };

    void onOpenGoals(const nervros_interfaces::msg::OpenGoals& goals);
    void check();
    void cancelAll(const nervros_interfaces::msg::OpenGoals& open);
    /// Sends the cancels whose server is up; drops those past their time with an error.
    void sendPending();
    /// Settles a cancel's answer: done, over already, or asked again.
    void onAnswer(const Pending& goal, std::int8_t code);
    /// Asks again for a cancel that was refused or not answered, while its time lasts.
    void again(const Pending& goal, const char* why);
    /// The cancel client of @p action, made the first time it is asked for.
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr& canceller(const std::string& action);

    std::chrono::nanoseconds                                                       timeout_;
    rclcpp::Subscription<nervros_interfaces::msg::OpenGoals>::SharedPtr            open_goals_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            moveit_stop_;
    rclcpp::TimerBase::SharedPtr                                                   timer_;
    std::map<std::string, rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> cancellers_;
    /// Each agent's open goals by its client name: one's list never stands in for another's.
    std::map<std::string, Agent> agents_;
    std::vector<Pending>         pending_;
    std::vector<Asked>           asked_;
};

}  // namespace nervros_watchdog

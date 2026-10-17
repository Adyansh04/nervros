#include "nervros_watchdog/watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nervros_watchdog
{

namespace
{
/// How often the agent's silence is checked.
constexpr std::chrono::milliseconds kCheckEvery{ 100 };
}  // namespace

Watchdog::Watchdog(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nervros_watchdog", options)
{
    const auto   topic = declare_parameter<std::string>("open_goals_topic", "/nervros/open_goals");
    const auto   stop  = declare_parameter<std::string>("moveit_stop_topic", "");
    const double limit = declare_parameter<double>("timeout_s", 2.0);
    if (!std::isfinite(limit) || limit <= 0.0)
    {
        throw std::invalid_argument("timeout_s must be above 0");
    }
    timeout_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(limit));

    // Latched, so a watchdog started after the agent still hears what is open.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    open_goals_        = create_subscription<nervros_interfaces::msg::OpenGoals>(
        topic,
        latched,
        [this](const nervros_interfaces::msg::OpenGoals& goals) { onOpenGoals(goals); });
    if (!stop.empty())
    {
        moveit_stop_ = create_publisher<std_msgs::msg::String>(stop, rclcpp::QoS(10).reliable());
    }
    timer_ = create_wall_timer(kCheckEvery, [this] { check(); });
    RCLCPP_INFO(
        get_logger(),
        "watching %s: goals still open %.1f s after it goes quiet are cancelled",
        topic.c_str(),
        limit);
}

void Watchdog::onOpenGoals(const nervros_interfaces::msg::OpenGoals& goals)
{
    open_  = goals;
    heard_ = std::chrono::steady_clock::now();
}

void Watchdog::check()
{
    if (open_.goal_ids.empty() || std::chrono::steady_clock::now() - heard_ < timeout_)
    {
        return;
    }
    cancelAll();
    open_ = nervros_interfaces::msg::OpenGoals();
}

void Watchdog::cancelAll()
{
    RCLCPP_WARN(
        get_logger(),
        "%s went quiet with %zu goals open: cancelling them",
        open_.client.empty() ? "the agent" : open_.client.c_str(),
        open_.goal_ids.size());
    const std::size_t n = std::min(open_.actions.size(), open_.goal_ids.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        const std::string& action = open_.actions[i];
        auto&              client = cancellers_[action];
        if (!client)
        {
            client = create_client<action_msgs::srv::CancelGoal>(action + "/_action/cancel_goal");
        }
        if (!client->service_is_ready())
        {
            RCLCPP_ERROR(
                get_logger(),
                "%s cannot be told to cancel: its server is gone",
                action.c_str());
            continue;
        }
        auto request               = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        request->goal_info.goal_id = open_.goal_ids[i];
        // A sent request is enough: waiting for the answer would block the timer.
        client->async_send_request(request);
        RCLCPP_WARN(get_logger(), "cancelled a goal of %s", action.c_str());
    }
    if (moveit_stop_)
    {
        std_msgs::msg::String stop;
        stop.data = "stop";
        moveit_stop_->publish(stop);
    }
}

}  // namespace nervros_watchdog

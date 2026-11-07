#include "nervros_watchdog/watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace nervros_watchdog
{

namespace
{
/// How often the agent's silence is checked.
constexpr std::chrono::milliseconds kCheckEvery{ 100 };
/// How long a cancel is retried while its server is not reachable: a client made a moment ago is
/// often not matched yet, and a server may be restarting.
constexpr std::chrono::seconds kRetryFor{ 5 };
/// How long a server may take to answer a cancel before it is asked again.
constexpr std::chrono::seconds kAnswerWait{ 2 };
}  // namespace

Watchdog::Watchdog(const rclcpp::NodeOptions& options)
  : rclcpp::Node("nervros_watchdog", options)
{
    const auto   topic = declare_parameter<std::string>("open_goals_topic", "/nervros/open_goals");
    const auto   stop  = declare_parameter<std::string>("moveit_stop_topic", "");
    const double limit = declare_parameter<double>("timeout_s", 2.0);
    // Shorter than a tenth of a second is no wait at all; past an hour it guards nothing.
    if (!std::isfinite(limit) || limit < 0.1 || limit > 3600.0)
    {
        throw std::invalid_argument(
            std::format("timeout_s must be from 0.1 to 3600 s, not {}", limit));
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
    // An empty list ends its own agent's entry only: a dead run's goals still time out.
    if (goals.goal_ids.empty())
    {
        agents_.erase(goals.client);
        return;
    }
    agents_[goals.client] = { goals, std::chrono::steady_clock::now() };
    // Made now, so they are matched long before an outage needs them.
    for (const std::string& action : goals.actions)
    {
        (void)canceller(action);
    }
}

rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr&
Watchdog::canceller(const std::string& action)
{
    auto& client = cancellers_[action];
    if (!client)
    {
        client = create_client<action_msgs::srv::CancelGoal>(action + "/_action/cancel_goal");
    }
    return client;
}

void Watchdog::check()
{
    const auto at = std::chrono::steady_clock::now();
    std::erase_if(asked_, [this, at](const Asked& asked) {
        if (at < asked.by)
        {
            return false;
        }
        canceller(asked.goal.action)->remove_pending_request(asked.request_id);
        again(asked.goal, "did not answer its cancel");
        return true;
    });
    sendPending();
    const auto now = std::chrono::steady_clock::now();
    std::erase_if(agents_, [this, now](const auto& entry) {
        if (now - entry.second.heard < timeout_)
        {
            return false;
        }
        cancelAll(entry.second.open);
        return true;
    });
}

void Watchdog::sendPending()
{
    const auto now = std::chrono::steady_clock::now();
    std::erase_if(pending_, [this, now](const Pending& goal) {
        auto& client = canceller(goal.action);
        if (client->service_is_ready())
        {
            auto request               = std::make_shared<action_msgs::srv::CancelGoal::Request>();
            request->goal_info.goal_id = goal.goal_id;
            // Answered later, so the timer is not blocked: a refusal or silence asks again.
            const auto sent = client->async_send_request(
                request,
                [this, goal](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture answer) {
                    onAnswer(goal, answer.get()->return_code);
                });
            asked_.push_back({ goal, sent.request_id, now + kAnswerWait });
            return true;
        }
        if (now >= goal.until)
        {
            RCLCPP_ERROR(
                get_logger(),
                "%s cannot be told to cancel: its server did not answer for %lld s",
                goal.action.c_str(),
                static_cast<long long>(kRetryFor.count()));
            return true;
        }
        return false;
    });
}

void Watchdog::onAnswer(const Pending& goal, std::int8_t code)
{
    std::erase_if(asked_, [&goal](const Asked& asked) {
        return asked.goal.action == goal.action && asked.goal.goal_id.uuid == goal.goal_id.uuid;
    });
    using Response = action_msgs::srv::CancelGoal::Response;
    if (code == Response::ERROR_NONE)
    {
        RCLCPP_WARN(get_logger(), "cancelled a goal of %s", goal.action.c_str());
    }
    else if (code == Response::ERROR_UNKNOWN_GOAL_ID || code == Response::ERROR_GOAL_TERMINATED)
    {
        RCLCPP_INFO(get_logger(), "a goal of %s had ended already", goal.action.c_str());
    }
    else
    {
        again(goal, "refused to cancel a goal");
    }
}

void Watchdog::again(const Pending& goal, const char* why)
{
    if (std::chrono::steady_clock::now() >= goal.until)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s %s, and its time to be asked is over: it may still be running",
            goal.action.c_str(),
            why);
        return;
    }
    RCLCPP_ERROR(get_logger(), "%s %s; asking again", goal.action.c_str(), why);
    pending_.push_back(goal);
}

void Watchdog::cancelAll(const nervros_interfaces::msg::OpenGoals& open)
{
    RCLCPP_WARN(
        get_logger(),
        "%s went quiet with %zu goals open: cancelling them",
        open.client.empty() ? "the agent" : open.client.c_str(),
        open.goal_ids.size());
    const std::size_t n     = std::min(open.actions.size(), open.goal_ids.size());
    const auto        until = std::chrono::steady_clock::now() + kRetryFor;
    for (std::size_t i = 0; i < n; ++i)
    {
        pending_.push_back({ open.actions[i], open.goal_ids[i], until });
    }
    sendPending();
    if (moveit_stop_)
    {
        std_msgs::msg::String stop;
        stop.data = "stop";
        moveit_stop_->publish(stop);
    }
}

}  // namespace nervros_watchdog

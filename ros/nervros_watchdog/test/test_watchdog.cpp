#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "nervros_watchdog/watchdog.hpp"

namespace
{

using nervros_interfaces::msg::OpenGoals;
using namespace std::chrono_literals;

/// A cancel service of an action that records the goals it is asked to cancel.
struct FakeAction
{
    explicit FakeAction(const std::string& action)
      : node(std::make_shared<rclcpp::Node>("fake_action"))
    {
        service = node->create_service<action_msgs::srv::CancelGoal>(
            action + "/_action/cancel_goal",
            [this](
                const action_msgs::srv::CancelGoal::Request::SharedPtr& request,
                const action_msgs::srv::CancelGoal::Response::SharedPtr&) {
                cancelled.push_back(request->goal_info.goal_id.uuid[0]);
            });
    }

    rclcpp::Node::SharedPtr                                  node;
    rclcpp::Service<action_msgs::srv::CancelGoal>::SharedPtr service;
    std::vector<int>                                         cancelled;
};

class WatchdogTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    /// A watchdog on `topic` that waits 0.3 s, the fake action, and a sender of open goals.
    void start(const std::string& topic, const std::string& action)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            { rclcpp::Parameter("open_goals_topic", topic), rclcpp::Parameter("timeout_s", 0.3) });
        watchdog = std::make_shared<nervros_watchdog::Watchdog>(options);
        fake     = std::make_unique<FakeAction>(action);
        agent    = std::make_shared<rclcpp::Node>("agent");
        sender =
            agent->create_publisher<OpenGoals>(topic, rclcpp::QoS(1).reliable().transient_local());
        executor.add_node(watchdog);
        executor.add_node(fake->node);
        executor.add_node(agent);
    }

    void spinFor(std::chrono::milliseconds span)
    {
        const auto end = std::chrono::steady_clock::now() + span;
        while (std::chrono::steady_clock::now() < end)
        {
            executor.spin_some();
            std::this_thread::sleep_for(5ms);
        }
    }

    static OpenGoals goals(const std::string& action, std::uint8_t id)
    {
        OpenGoals open;
        open.client = "nervros 1";
        open.actions.push_back(action);
        unique_identifier_msgs::msg::UUID uuid;
        uuid.uuid.fill(id);
        open.goal_ids.push_back(uuid);
        return open;
    }

    rclcpp::executors::SingleThreadedExecutor   executor;
    std::shared_ptr<nervros_watchdog::Watchdog> watchdog;
    std::unique_ptr<FakeAction>                 fake;
    rclcpp::Node::SharedPtr                     agent;
    rclcpp::Publisher<OpenGoals>::SharedPtr     sender;
};

TEST_F(WatchdogTest, GoalsOfAnAgentThatWentQuietAreCancelledOnce)
{
    start("/quiet/open_goals", "/quiet/walk");
    spinFor(200ms);
    sender->publish(goals("/quiet/walk", 7));
    spinFor(1000ms);
    ASSERT_EQ(fake->cancelled.size(), 1U);
    EXPECT_EQ(fake->cancelled[0], 7);
}

TEST_F(WatchdogTest, GoalsThatClosedOrKeepBeingSentAreLeftAlone)
{
    start("/alive/open_goals", "/alive/walk");
    spinFor(200ms);
    for (int i = 0; i < 8; ++i)
    {
        sender->publish(goals("/alive/walk", 3));
        spinFor(100ms);
    }
    sender->publish(OpenGoals());
    spinFor(700ms);
    EXPECT_TRUE(fake->cancelled.empty());
}

}  // namespace

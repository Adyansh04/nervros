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

/// A cancel service of an action that records the goals it is asked to cancel, and refuses the
/// first `refusals` of them.
struct FakeAction
{
    explicit FakeAction(const std::string& action, int refusals = 0)
      : node(std::make_shared<rclcpp::Node>("fake_action"))
      , refusals_left(refusals)
    {
        service = node->create_service<action_msgs::srv::CancelGoal>(
            action + "/_action/cancel_goal",
            [this](
                const action_msgs::srv::CancelGoal::Request::SharedPtr&  request,
                const action_msgs::srv::CancelGoal::Response::SharedPtr& response) {
                cancelled.push_back(request->goal_info.goal_id.uuid[0]);
                if (refusals_left > 0)
                {
                    --refusals_left;
                    response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_REJECTED;
                }
            });
    }

    rclcpp::Node::SharedPtr                                  node;
    rclcpp::Service<action_msgs::srv::CancelGoal>::SharedPtr service;
    std::vector<int>                                         cancelled;
    int                                                      refusals_left = 0;
};

class WatchdogTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    /// A watchdog on `topic` that waits 0.3 s, the fake action unless it comes `late`, and a
    /// sender of open goals.
    void
    start(const std::string& topic, const std::string& action, int refusals = 0, bool late = false)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            { rclcpp::Parameter("open_goals_topic", topic), rclcpp::Parameter("timeout_s", 0.3) });
        watchdog = std::make_shared<nervros_watchdog::Watchdog>(options);
        agent    = std::make_shared<rclcpp::Node>("agent");
        sender =
            agent->create_publisher<OpenGoals>(topic, rclcpp::QoS(1).reliable().transient_local());
        executor.add_node(watchdog);
        executor.add_node(agent);
        if (!late)
        {
            serve(action, refusals);
        }
    }

    /// Brings the action's server up.
    void serve(const std::string& action, int refusals = 0)
    {
        fake = std::make_unique<FakeAction>(action, refusals);
        executor.add_node(fake->node);
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
    OpenGoals closed;
    closed.client = "nervros 1";
    sender->publish(closed);
    spinFor(700ms);
    EXPECT_TRUE(fake->cancelled.empty());
}

TEST_F(WatchdogTest, AnotherAgentsEmptyListDoesNotHideAQuietOnesGoals)
{
    start("/two/open_goals", "/two/walk");
    spinFor(200ms);
    sender->publish(goals("/two/walk", 5));
    spinFor(50ms);
    // A NervROS started again says it has nothing open; the run before it still had a walk.
    OpenGoals fresh;
    fresh.client = "nervros 2";
    sender->publish(fresh);
    spinFor(1000ms);
    ASSERT_EQ(fake->cancelled.size(), 1U);
    EXPECT_EQ(fake->cancelled[0], 5);
}

TEST_F(WatchdogTest, ARefusedCancelIsAskedAgain)
{
    start("/refused/open_goals", "/refused/walk", 1);
    spinFor(200ms);
    sender->publish(goals("/refused/walk", 4));
    spinFor(1500ms);
    ASSERT_EQ(fake->cancelled.size(), 2U) << "refused once, then asked again";
    EXPECT_EQ(fake->cancelled[1], 4);
}

TEST_F(WatchdogTest, AServerThatComesUpLateIsStillAsked)
{
    start("/late/open_goals", "/late/walk", 0, true);
    spinFor(200ms);
    sender->publish(goals("/late/walk", 9));
    spinFor(800ms);
    serve("/late/walk");
    spinFor(1500ms);
    ASSERT_EQ(fake->cancelled.size(), 1U);
    EXPECT_EQ(fake->cancelled[0], 9);
}

}  // namespace

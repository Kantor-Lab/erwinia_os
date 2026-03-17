#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <chrono>
#include <thread>

class NavigationServer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  NavigationServer()
  : Node("navigation_server")
  {
    declare_parameter<int>("simulate_delay_ms", 5);

    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "navigation_server",
      std::bind(&NavigationServer::handle_goal, this, _1, _2),
      std::bind(&NavigationServer::handle_cancel, this, _1),
      std::bind(&NavigationServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Navigation action server ready: navigation_server");
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received navigation goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&NavigationServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const int delay_ms = get_parameter("simulate_delay_ms").as_int();
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    if (goal_handle->is_canceling())
    {
      auto result = std::make_shared<NavigateToPose::Result>();
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Navigation goal canceled");
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Navigation goal succeeded");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

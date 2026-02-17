#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <erwinia_msgs/action/mark_location.hpp>

#include <chrono>
#include <thread>

class MarkServer : public rclcpp::Node
{
public:
  using MarkLocation = erwinia_msgs::action::MarkLocation;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MarkLocation>;

  MarkServer()
  : Node("mark_server")
  {
    declare_parameter<int>("simulate_delay_ms", 5);
    declare_parameter<bool>("marked", true);
    declare_parameter<std::string>("error", "");

    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<MarkLocation>(
      this,
      "mark_server",
      std::bind(&MarkServer::handle_goal, this, _1, _2),
      std::bind(&MarkServer::handle_cancel, this, _1),
      std::bind(&MarkServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Mark action server ready: mark_server");
  }

private:
  rclcpp_action::Server<MarkLocation>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MarkLocation::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received mark goal");
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
    std::thread{std::bind(&MarkServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const int delay_ms = get_parameter("simulate_delay_ms").as_int();
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    if (goal_handle->is_canceling())
    {
      auto result = std::make_shared<MarkLocation::Result>();
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Mark goal canceled");
      return;
    }

    auto result = std::make_shared<MarkLocation::Result>();
    result->marked = get_parameter("marked").as_bool();
    result->error = get_parameter("error").as_string();
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Mark goal succeeded (marked=%s)",
                result->marked ? "true" : "false");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarkServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

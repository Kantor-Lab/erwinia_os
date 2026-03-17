#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <erwinia_msgs/action/detect_erwinia.hpp>

#include <chrono>
#include <thread>

class ErwiniaDetectorServer : public rclcpp::Node
{
public:
  using DetectErwinia = erwinia_msgs::action::DetectErwinia;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DetectErwinia>;

  ErwiniaDetectorServer()
  : Node("erwinia_detector_server")
  {
    declare_parameter<int>("simulate_delay_ms", 5);
    declare_parameter<bool>("infected", false);

    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<DetectErwinia>(
      this,
      "erwinia_detector_server",
      std::bind(&ErwiniaDetectorServer::handle_goal, this, _1, _2),
      std::bind(&ErwiniaDetectorServer::handle_cancel, this, _1),
      std::bind(&ErwiniaDetectorServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Erwinia detector action server ready: erwinia_detector_server");
  }

private:
  rclcpp_action::Server<DetectErwinia>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const DetectErwinia::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received detection goal");
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
    std::thread{std::bind(&ErwiniaDetectorServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const int delay_ms = get_parameter("simulate_delay_ms").as_int();
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    if (goal_handle->is_canceling())
    {
      auto result = std::make_shared<DetectErwinia::Result>();
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Detection goal canceled");
      return;
    }

    auto result = std::make_shared<DetectErwinia::Result>();
    // ask user for result (infected or not)
    std::string input;
    std::cout << "Is the plant infected? (y/n): ";
    std::cin >> input;
    result->infected = (input == "y");
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Detection goal succeeded (infected=%s)",
                result->infected ? "true" : "false");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ErwiniaDetectorServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

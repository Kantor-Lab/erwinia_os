#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <erwinia_os_nbv_planner/action/run_nbv.hpp>
#include <erwinia_msgs/action/mark_location.hpp>

#include <chrono>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace erwinia_bt_nodes
{

static bool interactiveEnabled()
{
  const char * value = std::getenv("BT_INTERACTIVE");
  return (value == nullptr) || (std::string(value) != "0");
}

static BT::NodeStatus parseInteractiveStatus(const std::string & input, BT::NodeStatus default_status)
{
  if (input.empty())
  {
    return default_status;
  }

  const char choice = static_cast<char>(std::tolower(input.front()));
  if (choice == 's')
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (choice == 'f')
  {
    return BT::NodeStatus::FAILURE;
  }
  return default_status;
}

enum class InteractiveNavOutcome
{
  None,
  Success,
  Failure
};

template<typename ParameterT>
static ParameterT declareOrGetParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name))
  {
    return node->get_parameter(name).get_value<ParameterT>();
  }

  return node->declare_parameter<ParameterT>(name, default_value);
}

static bool parsePoseString(const std::string& text, geometry_msgs::msg::PoseStamped& out)
{
  // Accept "x,y" or "x,y,frame"
  std::stringstream ss(text);
  std::string item;
  if (!std::getline(ss, item, ','))
  {
    return false;
  }
  try
  {
    out.pose.position.x = std::stod(item);
  }
  catch (...)
  {
    return false;
  }
  if (!std::getline(ss, item, ','))
  {
    return false;
  }
  try
  {
    out.pose.position.y = std::stod(item);
  }
  catch (...)
  {
    return false;
  }
  out.pose.position.z = 0.0;
  out.pose.orientation.w = 1.0;

  if (std::getline(ss, item, ','))
  {
    out.header.frame_id = item;
  }
  else
  {
    out.header.frame_id = "map";
  }
  return true;
}

static geometry_msgs::msg::PoseStamped defaultPose(const rclcpp::Node::SharedPtr& node)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class NavigateToPoseNode : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    if (!config.blackboard)
    {
      throw BT::RuntimeError("NavigateToPoseNode: missing blackboard");
    }
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_)
    {
      throw BT::RuntimeError("NavigateToPoseNode: blackboard has no rclcpp::Node");
    }

    joy_topic_ = declareOrGetParameter<std::string>(node_, "interactive_joy_topic", "joy");
    button_success_ = declareOrGetParameter<int>(node_, "interactive_button_success", 0);
    button_failure_ = declareOrGetParameter<int>(node_, "interactive_button_failure", 3);
    joy_poll_ms_ = declareOrGetParameter<int>(node_, "interactive_joy_poll_ms", 50);
    const std::string device =
      declareOrGetParameter<std::string>(node_, "interactive_input_device", "keyboard");
    interactive_xbox_ =
      (device == "xbox" || device == "controller" || device == "xbox_controller");

    if (interactiveEnabled() && interactive_xbox_)
    {
      joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) { onJoy(msg); });
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
      BT::InputPort<std::string>("goal"),
      BT::InputPort<std::string>("server_name", "navigation_server"),
      BT::InputPort<int>("server_timeout_ms", 2000, "")
    };
  }

  BT::NodeStatus onStart() override
  {
    resetState();

    auto goal_text = getInput<std::string>("goal");
    if (goal_text)
    {
      RCLCPP_INFO(node_->get_logger(), "NavigateToPose target: %s", goal_text.value().c_str());
    }

    if (interactiveEnabled())
    {
      if (interactive_xbox_)
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "Interactive navigation enabled. Teleoperate the robot to the target, then press "
          "success button %d or failure button %d on topic '%s'.",
          button_success_, button_failure_, joy_topic_.c_str());
      }
      else
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "Interactive navigation enabled without xbox input. Navigate manually and use the "
          "keyboard prompt to mark success or failure.");
      }
      return BT::NodeStatus::RUNNING;
    }

    std::string server_name = "navigation_server";
    getInput("server_name", server_name);

    if (!client_ || server_name != server_name_)
    {
      server_name_ = server_name;
      client_ = rclcpp_action::create_client<NavigateToPose>(node_, server_name_);
    }

    int timeout_ms = 2000;
    getInput("server_timeout_ms", timeout_ms);
    if (!client_->wait_for_action_server(std::chrono::milliseconds(timeout_ms)))
    {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available: %s",
                   server_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped pose;
    bool have_pose = false;
    if (auto pose_in = getInput<geometry_msgs::msg::PoseStamped>("pose"))
    {
      pose = pose_in.value();
      have_pose = true;
    }
    else if (auto goal_text = getInput<std::string>("goal"))
    {
      if (parsePoseString(goal_text.value(), pose))
      {
        have_pose = true;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(),
                    "NavigateToPose goal string not parseable, using default pose");
        pose = defaultPose(node_);
        have_pose = true;
      }
    }

    if (!have_pose)
    {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose missing goal input");
      return BT::NodeStatus::FAILURE;
    }

    pose.header.stamp = node_->now();

    NavigateToPose::Goal goal;
    goal.pose = pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_response_received_ = true;
        goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        result_code_ = result.code;
        result_ready_ = true;
      };

    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (interactiveEnabled())
    {
      if (!interactive_xbox_)
      {
        std::string line;
        std::cout << "[NavigateToPose] status? (s=success, f=failure, enter=running) > "
                  << std::flush;
        std::getline(std::cin, line);
        return parseInteractiveStatus(line, BT::NodeStatus::RUNNING);
      }

      rclcpp::spin_some(node_);

      InteractiveNavOutcome outcome = InteractiveNavOutcome::None;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        outcome = interactive_outcome_;
        interactive_outcome_ = InteractiveNavOutcome::None;
      }

      if (outcome == InteractiveNavOutcome::Success)
      {
        RCLCPP_INFO(node_->get_logger(), "Interactive navigation marked success");
        return BT::NodeStatus::SUCCESS;
      }
      if (outcome == InteractiveNavOutcome::Failure)
      {
        RCLCPP_WARN(node_->get_logger(), "Interactive navigation marked failure");
        return BT::NodeStatus::FAILURE;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(joy_poll_ms_));
      return BT::NodeStatus::RUNNING;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (goal_response_received_ && !goal_handle_)
    {
      return BT::NodeStatus::FAILURE;
    }

    if (result_ready_)
    {
      return (result_code_ == rclcpp_action::ResultCode::SUCCEEDED)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_)
    {
      client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  void resetState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    result_ready_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    interactive_outcome_ = InteractiveNavOutcome::None;
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (previous_buttons_.size() != msg->buttons.size())
    {
      previous_buttons_.assign(msg->buttons.size(), 0);
    }

    const auto detect_edge = [&](int index) -> bool {
      return index >= 0 &&
             static_cast<size_t>(index) < msg->buttons.size() &&
             msg->buttons[index] != 0 &&
             previous_buttons_[index] == 0;
    };

    if (detect_edge(button_success_))
    {
      interactive_outcome_ = InteractiveNavOutcome::Success;
    }
    else if (detect_edge(button_failure_))
    {
      interactive_outcome_ = InteractiveNavOutcome::Failure;
    }

    previous_buttons_ = msg->buttons;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::string server_name_;
  std::string joy_topic_{"joy"};
  int button_success_{0};
  int button_failure_{3};
  int joy_poll_ms_{50};
  bool interactive_xbox_{false};

  std::mutex mutex_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_response_received_{false};
  bool result_ready_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
  std::vector<int32_t> previous_buttons_;
  InteractiveNavOutcome interactive_outcome_{InteractiveNavOutcome::None};
};

class ErwiniaDetectorNode : public BT::StatefulActionNode
{
public:
  using RunNBV = erwinia_os_nbv_planner::action::RunNBV;
  using GoalHandle = rclcpp_action::ClientGoalHandle<RunNBV>;

  ErwiniaDetectorNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    if (!config.blackboard)
    {
      throw BT::RuntimeError("ErwiniaDetectorNode: missing blackboard");
    }
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_)
    {
      throw BT::RuntimeError("ErwiniaDetectorNode: blackboard has no rclcpp::Node");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("planner_type", "baseline"),
      BT::OutputPort<std::string>("result"),
      BT::InputPort<std::string>("server_name", "run_nbv"),
      BT::InputPort<int>("server_timeout_ms", 2000, "")
    };
  }

  BT::NodeStatus onStart() override
  {
    resetState();

    std::string server_name = "run_nbv";
    getInput("server_name", server_name);

    if (!client_ || server_name != server_name_)
    {
      server_name_ = server_name;
      client_ = rclcpp_action::create_client<RunNBV>(node_, server_name_);
    }

    int timeout_ms = 2000;
    getInput("server_timeout_ms", timeout_ms);
    if (!client_->wait_for_action_server(std::chrono::milliseconds(timeout_ms)))
    {
      RCLCPP_ERROR(node_->get_logger(), "RunNBV action server not available: %s",
                   server_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    std::string planner_type = "baseline";
    getInput("planner_type", planner_type);

    RunNBV::Goal goal;
    goal.planner_type = planner_type;

    auto options = rclcpp_action::Client<RunNBV>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_response_received_ = true;
        goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        result_code_ = result.code;
        if (result.result)
        {
          final_cluster_count_ = result.result->final_cluster_count;
        }
        result_ready_ = true;
      };

    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (goal_response_received_ && !goal_handle_)
    {
      return BT::NodeStatus::FAILURE;
    }

    if (result_ready_)
    {
      setOutput("result", final_cluster_count_ > 0 ? "infected" : "healthy");
      return (result_code_ == rclcpp_action::ResultCode::SUCCEEDED)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_)
    {
      client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  void resetState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    result_ready_ = false;
    final_cluster_count_ = 0;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<RunNBV>::SharedPtr client_;
  std::string server_name_;

  std::mutex mutex_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_response_received_{false};
  bool result_ready_{false};
  int32_t final_cluster_count_{0};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
};

class MarkLocationNode : public BT::StatefulActionNode
{
public:
  using MarkLocation = erwinia_msgs::action::MarkLocation;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MarkLocation>;

  MarkLocationNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    if (!config.blackboard)
    {
      throw BT::RuntimeError("MarkLocationNode: missing blackboard");
    }
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_)
    {
      throw BT::RuntimeError("MarkLocationNode: blackboard has no rclcpp::Node");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target"),
      BT::OutputPort<bool>("marked"),
      BT::OutputPort<std::string>("error"),
      BT::InputPort<std::string>("server_name", "mark_server"),
      BT::InputPort<int>("server_timeout_ms", 2000, "")
    };
  }

  BT::NodeStatus onStart() override
  {
    resetState();

    std::string server_name = "mark_server";
    getInput("server_name", server_name);

    if (!client_ || server_name != server_name_)
    {
      server_name_ = server_name;
      client_ = rclcpp_action::create_client<MarkLocation>(node_, server_name_);
    }

    int timeout_ms = 2000;
    getInput("server_timeout_ms", timeout_ms);
    if (!client_->wait_for_action_server(std::chrono::milliseconds(timeout_ms)))
    {
      RCLCPP_ERROR(node_->get_logger(), "MarkLocation action server not available: %s",
                   server_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    MarkLocation::Goal goal;

    auto options = rclcpp_action::Client<MarkLocation>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_response_received_ = true;
        goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        result_code_ = result.code;
        if (result.result)
        {
          marked_ = result.result->marked;
          error_ = result.result->error;
        }
        result_ready_ = true;
      };

    client_->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (goal_response_received_ && !goal_handle_)
    {
      return BT::NodeStatus::FAILURE;
    }

    if (result_ready_)
    {
      setOutput("marked", marked_);
      setOutput("error", error_);
      return (result_code_ == rclcpp_action::ResultCode::SUCCEEDED)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_)
    {
      client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  void resetState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    result_ready_ = false;
    marked_ = false;
    error_.clear();
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MarkLocation>::SharedPtr client_;
  std::string server_name_;

  std::mutex mutex_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_response_received_{false};
  bool result_ready_{false};
  bool marked_{false};
  std::string error_;
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
};

}  // namespace erwinia_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<erwinia_bt_nodes::NavigateToPoseNode>("NavigateToPose");
  factory.registerNodeType<erwinia_bt_nodes::ErwiniaDetectorNode>("ErwiniaDetector");
  factory.registerNodeType<erwinia_bt_nodes::MarkLocationNode>("MarkLocation");
}

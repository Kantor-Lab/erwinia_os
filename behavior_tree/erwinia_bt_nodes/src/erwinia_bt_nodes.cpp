#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <erwinia_os_nbv_planner/action/run_nbv.hpp>
#include <erwinia_msgs/action/mark_location.hpp>

#include <chrono>
#include <cctype>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace erwinia_bt_nodes
{

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
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::string server_name_;

  std::mutex mutex_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_response_received_{false};
  bool result_ready_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
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

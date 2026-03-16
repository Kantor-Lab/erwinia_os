#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <chrono>
#include <cstdlib>
#include <cctype>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace erwinia_bt
{

inline bool interactiveEnabled()
{
  static const bool enabled = []() {
    const char* v = std::getenv("BT_INTERACTIVE");
    return (v == nullptr) || (std::string(v) != "0");
  }();
  return enabled;
}

inline std::string readLine()
{
  std::string line;
  std::getline(std::cin, line);
  return line;
}

enum class InteractiveDevice
{
  Keyboard,
  XboxController
};

enum class InteractiveChoice
{
  Auto,
  Success,
  Failure,
  Running
};

class InteractiveInput
{
public:
  static InteractiveInput& instance()
  {
    static InteractiveInput input;
    return input;
  }

  void configure(const rclcpp::Node::SharedPtr& node)
  {
    if (!node)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (node_ == node)
    {
      return;
    }

    node_ = node;

    const std::string device =
      node_->declare_parameter<std::string>("interactive_input_device", "keyboard");
    joy_topic_ = node_->declare_parameter<std::string>("interactive_joy_topic", "joy");
    button_auto_ = node_->declare_parameter<int>("interactive_button_auto", 4);
    button_success_ = node_->declare_parameter<int>("interactive_button_success", 0);
    button_failure_ = node_->declare_parameter<int>("interactive_button_failure", 3);
    button_running_ = node_->declare_parameter<int>("interactive_button_running", 1);
    poll_ms_ = node_->declare_parameter<int>("interactive_joy_poll_ms", 50);

    device_ = (device == "xbox" || device == "controller" || device == "xbox_controller")
                ? InteractiveDevice::XboxController
                : InteractiveDevice::Keyboard;

    joy_sub_.reset();
    previous_buttons_.clear();
    has_pending_choice_ = false;

    if (device_ == InteractiveDevice::XboxController)
    {
      joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) { onJoy(msg); });

      RCLCPP_INFO(
        node_->get_logger(),
        "Interactive input: xbox controller on topic '%s' (auto=%d success=%d failure=%d running=%d)",
        joy_topic_.c_str(), button_auto_, button_success_, button_failure_, button_running_);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Interactive input: keyboard");
    }
  }

  bool isController() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return device_ == InteractiveDevice::XboxController;
  }

  std::string readCommand(const std::string& prompt, bool allow_running)
  {
    if (!interactiveEnabled())
    {
      return "";
    }

    if (isController())
    {
      return waitForControllerCommand(prompt, allow_running);
    }

    std::cout << prompt << std::flush;
    return readLine();
  }

private:
  std::string waitForControllerCommand(const std::string& prompt, bool allow_running)
  {
    rclcpp::Node::SharedPtr node;
    int poll_ms = 50;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      node = node_;
      poll_ms = poll_ms_;
      has_pending_choice_ = false;
    }

    if (!node)
    {
      return "";
    }

    std::cout << prompt
              << " [controller: auto/success/failure"
              << (allow_running ? "/running" : "")
              << "] > " << std::flush;

    while (rclcpp::ok())
    {
      rclcpp::spin_some(node);

      bool has_choice = false;
      InteractiveChoice choice = InteractiveChoice::Auto;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_pending_choice_)
        {
          choice = pending_choice_;
          has_pending_choice_ = false;
          has_choice = true;
        }
      }

      if (has_choice)
      {
        switch (choice)
        {
          case InteractiveChoice::Auto:
            std::cout << "auto" << std::endl;
            return "a";
          case InteractiveChoice::Success:
            std::cout << "success" << std::endl;
            return "s";
          case InteractiveChoice::Failure:
            std::cout << "failure" << std::endl;
            return "f";
          case InteractiveChoice::Running:
            if (allow_running)
            {
              std::cout << "running" << std::endl;
              return "r";
            }
            break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }

    return "";
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

    if (detect_edge(button_auto_))
    {
      pending_choice_ = InteractiveChoice::Auto;
      has_pending_choice_ = true;
    }
    else if (detect_edge(button_success_))
    {
      pending_choice_ = InteractiveChoice::Success;
      has_pending_choice_ = true;
    }
    else if (detect_edge(button_failure_))
    {
      pending_choice_ = InteractiveChoice::Failure;
      has_pending_choice_ = true;
    }
    else if (detect_edge(button_running_))
    {
      pending_choice_ = InteractiveChoice::Running;
      has_pending_choice_ = true;
    }

    previous_buttons_ = msg->buttons;
  }

  mutable std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  InteractiveDevice device_{InteractiveDevice::Keyboard};
  std::string joy_topic_{"joy"};
  int button_auto_{0};
  int button_success_{1};
  int button_failure_{2};
  int button_running_{3};
  int poll_ms_{50};
  std::vector<int32_t> previous_buttons_;
  InteractiveChoice pending_choice_{InteractiveChoice::Auto};
  bool has_pending_choice_{false};
};

inline void configureInteractiveInput(const BT::NodeConfiguration& config)
{
  if (!config.blackboard)
  {
    return;
  }

  try
  {
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    InteractiveInput::instance().configure(node);
  }
  catch (...)
  {
  }
}

inline BT::NodeStatus parseStatus(const std::string& input, BT::NodeStatus def, bool allow_running)
{
  if (input.empty())
  {
    return def;
  }
  const char c = static_cast<char>(std::tolower(input[0]));
  if (c == 's') return BT::NodeStatus::SUCCESS;
  if (c == 'f') return BT::NodeStatus::FAILURE;
  if (c == 'r') return allow_running ? BT::NodeStatus::RUNNING : def;
  return def;
}

inline BT::NodeStatus promptStatus(const std::string& node_name,
                                   BT::NodeStatus def,
                                   bool allow_running)
{
  if (!interactiveEnabled())
  {
    return def;
  }
  const std::string prompt = allow_running
                               ? "[" + node_name + "] status? (s=success, f=failure, r=running, enter=default) > "
                               : "[" + node_name + "] status? (s=success, f=failure, enter=default) > ";
  const std::string line = InteractiveInput::instance().readCommand(prompt, allow_running);
  return parseStatus(line, def, allow_running);
}

inline std::vector<std::string> parseList(const std::string& input,
                                          const std::vector<std::string>& def)
{
  if (input.empty())
  {
    return def;
  }
  std::vector<std::string> out;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ','))
  {
    if (!item.empty())
    {
      out.push_back(item);
    }
  }
  return out.empty() ? def : out;
}

class InitSystem : public BT::StatefulActionNode
{
public:
  InitSystem(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    std::cout << "[InitSystem] init ok" << std::endl;
    return promptStatus("InitSystem", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("InitSystem", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class ShutdownSystem : public BT::StatefulActionNode
{
public:
  ShutdownSystem(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    std::cout << "[ShutdownSystem] shutdown ok" << std::endl;
    return promptStatus("ShutdownSystem", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("ShutdownSystem", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class LoadTargets : public BT::StatefulActionNode
{
public:
  LoadTargets(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::vector<std::string>>("targets")};
  }

  BT::NodeStatus onStart() override
  {
    std::vector<std::string> targets = {"tree_1", "tree_2", "tree_3"};
    if (interactiveEnabled())
    {
      std::cout << "[LoadTargets] targets (comma-separated, enter=default tree_1,tree_2,tree_3) > "
                << std::flush;
      targets = parseList(InteractiveInput::instance().readCommand(
                            "[LoadTargets] targets (comma-separated, enter=default tree_1,tree_2,tree_3)",
                            false),
                          targets);
    }
    setOutput("targets", targets);
    std::cout << "[LoadTargets] loaded " << targets.size() << " targets" << std::endl;
    return promptStatus("LoadTargets", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("LoadTargets", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class ForEachTarget : public BT::ControlNode
{
public:
  ForEachTarget(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ControlNode(name, config), index_(0)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<std::string>>("targets"),
            BT::OutputPort<std::string>("current")};
  }

  BT::NodeStatus tick() override
  {
    if (interactiveEnabled())
    {
      const std::string line = InteractiveInput::instance().readCommand(
        "[ForEachTarget] mode? (a=auto, s=success, f=failure, r=running)", true);
      const char c = line.empty() ? 'a' : static_cast<char>(std::tolower(line[0]));
      if (c == 's') return BT::NodeStatus::SUCCESS;
      if (c == 'f') return BT::NodeStatus::FAILURE;
      if (c == 'r') return BT::NodeStatus::RUNNING;
    }

    if (children_nodes_.size() != 1)
    {
      throw BT::LogicError("ForEachTarget must have exactly 1 child");
    }

    if (status() == BT::NodeStatus::IDLE)
    {
      if (!getInput("targets", targets_))
      {
        std::cout << "[ForEachTarget] missing targets input" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      index_ = 0;
    }

    while (index_ < targets_.size())
    {
      setOutput("current", targets_[index_]);
      auto child_status = children_nodes_[0]->executeTick();

      if (child_status == BT::NodeStatus::SUCCESS)
      {
        children_nodes_[0]->halt();
        ++index_;
        continue;
      }
      if (child_status == BT::NodeStatus::FAILURE)
      {
        children_nodes_[0]->halt();
        ++index_;          // skip this one
        continue;          // move to next target
        //return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    index_ = 0;
    BT::ControlNode::halt();
  }

private:
  size_t index_;
  std::vector<std::string> targets_;
};

class ReachedPose : public BT::ConditionNode
{
public:
  ReachedPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal"),
            BT::InputPort<double>("tol")};
  }

  BT::NodeStatus tick() override
  {
    auto goal = getInput<std::string>("goal");
    if (!goal)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (interactiveEnabled())
    {
      return promptStatus("ReachedPose", BT::NodeStatus::FAILURE, false);
    }
    return (goal.value().find("reached") != std::string::npos)
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
  }
};

class NavigateToPose : public BT::StatefulActionNode
{
public:
  NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal")};
  }

  BT::NodeStatus onStart() override
  {
    auto goal = getInput<std::string>("goal");
    if (goal)
    {
      std::cout << "[NavigateToPose] navigating to " << goal.value() << std::endl;
    }
    return promptStatus("NavigateToPose", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("NavigateToPose", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class ErwiniaDetector : public BT::StatefulActionNode
{
public:
  ErwiniaDetector(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target"),
            BT::OutputPort<std::string>("result")};
  }

  BT::NodeStatus onStart() override
  {
    auto target = getInput<std::string>("target");
    if (!target)
    {
      return BT::NodeStatus::FAILURE;
    }
    std::string result = (target.value().size() % 2) == 0 ? "infected" : "healthy";
    if (interactiveEnabled())
    {
      std::cout << "[ErwiniaDetector] result? (infected/healthy, enter=default) > "
                << std::flush;
      const std::string line = InteractiveInput::instance().readCommand(
        "[ErwiniaDetector] result? (infected/healthy, enter=default)", false);
      if (!line.empty())
      {
        result = line;
      }
    }
    setOutput("result", result);
    std::cout << "[ErwiniaDetector] " << target.value()
              << " -> " << result << std::endl;
    return promptStatus("ErwiniaDetector", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("ErwiniaDetector", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class IsInfected : public BT::ConditionNode
{
public:
  IsInfected(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("result")};
  }

  BT::NodeStatus tick() override
  {
    auto result = getInput<std::string>("result");
    if (!result)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (interactiveEnabled())
    {
      const std::string line = InteractiveInput::instance().readCommand(
        "[IsInfected] mode? (a=auto, s=success, f=failure)", false);
      const char c = line.empty() ? 'a' : static_cast<char>(std::tolower(line[0]));
      if (c == 's') return BT::NodeStatus::SUCCESS;
      if (c == 'f') return BT::NodeStatus::FAILURE;
    }
    return (result.value() == "infected") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ComputeMarkPose : public BT::StatefulActionNode
{
public:
  ComputeMarkPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target"),
            BT::OutputPort<std::string>("mark")};
  }

  BT::NodeStatus onStart() override
  {
    auto target = getInput<std::string>("target");
    if (!target)
    {
      return BT::NodeStatus::FAILURE;
    }
    std::string pose = "mark_" + target.value();
    if (interactiveEnabled())
    {
      std::cout << "[ComputeMarkPose] mark pose? (enter=default " << pose << ") > "
                << std::flush;
      const std::string line = InteractiveInput::instance().readCommand(
        "[ComputeMarkPose] mark pose? (enter=default " + pose + ")", false);
      if (!line.empty())
      {
        pose = line;
      }
    }
    setOutput("mark", pose);
    std::cout << "[ComputeMarkPose] " << target.value() << " -> " << pose << std::endl;
    return promptStatus("ComputeMarkPose", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("ComputeMarkPose", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class MarkLocation : public BT::StatefulActionNode
{
public:
  MarkLocation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target"),
            BT::OutputPort<bool>("marked"),
            BT::OutputPort<std::string>("error")};
  }

  BT::NodeStatus onStart() override
  {
    auto target = getInput<std::string>("target");
    if (target)
    {
      std::cout << "[MarkLocation] marked " << target.value() << std::endl;
    }
    bool marked = true;
    std::string error = "";
    if (interactiveEnabled())
    {
      std::cout << "[MarkLocation] marked? (true/false, enter=default true) > "
                << std::flush;
      const std::string marked_line = InteractiveInput::instance().readCommand(
        "[MarkLocation] marked? (true/false, enter=default true)", false);
      if (!marked_line.empty())
      {
        marked = (marked_line == "true" || marked_line == "1" || marked_line == "yes");
      }
      error = InteractiveInput::instance().readCommand(
        "[MarkLocation] error? (enter=default empty)", false);
    }
    setOutput("marked", marked);
    setOutput("error", error);
    return promptStatus("MarkLocation", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("MarkLocation", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

class LogTreeResult : public BT::StatefulActionNode
{
public:
  LogTreeResult(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    configureInteractiveInput(config);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target"),
            BT::InputPort<std::string>("result"),
            BT::InputPort<bool>("marked"),
            BT::InputPort<std::string>("mark_error")};
  }

  BT::NodeStatus onStart() override
  {
    auto target = getInput<std::string>("target");
    auto result = getInput<std::string>("result");
    auto marked = getInput<bool>("marked");
    auto error = getInput<std::string>("mark_error");

    std::cout << "[LogTreeResult] target=" << (target ? target.value() : "?")
              << " result=" << (result ? result.value() : "?")
              << " marked=" << (marked ? (marked.value() ? "true" : "false") : "?")
              << " error=" << (error ? error.value() : "") << std::endl;
    return promptStatus("LogTreeResult", BT::NodeStatus::SUCCESS, true);
  }

  BT::NodeStatus onRunning() override
  {
    return promptStatus("LogTreeResult", BT::NodeStatus::RUNNING, true);
  }

  void onHalted() override {}
};

}  // namespace erwinia_bt

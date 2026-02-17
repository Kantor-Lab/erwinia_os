#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/basic_types.h>

#include <cstdlib>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
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
  if (allow_running)
  {
    std::cout << "[" << node_name << "] status? (s=success, f=failure, r=running, enter=default) > "
              << std::flush;
  }
  else
  {
    std::cout << "[" << node_name << "] status? (s=success, f=failure, enter=default) > "
              << std::flush;
  }
  const std::string line = readLine();
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
      targets = parseList(readLine(), targets);
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
      std::cout << "[ForEachTarget] mode? (a=auto, s=success, f=failure, r=running) > "
                << std::flush;
      const std::string line = readLine();
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
      const std::string line = readLine();
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
      std::cout << "[IsInfected] mode? (a=auto, s=success, f=failure) > " << std::flush;
      const std::string line = readLine();
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
      const std::string line = readLine();
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
      const std::string marked_line = readLine();
      if (!marked_line.empty())
      {
        marked = (marked_line == "true" || marked_line == "1" || marked_line == "yes");
      }
      std::cout << "[MarkLocation] error? (enter=default empty) > " << std::flush;
      error = readLine();
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

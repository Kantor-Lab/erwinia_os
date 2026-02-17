#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dummy_nodes.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("BT_executor");

  BT::BehaviorTreeFactory factory;

  // Always register local utility/flow nodes
  factory.registerNodeType<erwinia_bt::InitSystem>("InitSystem");
  factory.registerNodeType<erwinia_bt::LoadTargets>("LoadTargets");
  factory.registerNodeType<erwinia_bt::ForEachTarget>("ForEachTarget");
  factory.registerNodeType<erwinia_bt::ReachedPose>("ReachedPose");
  factory.registerNodeType<erwinia_bt::IsInfected>("IsInfected");
  factory.registerNodeType<erwinia_bt::ComputeMarkPose>("ComputeMarkPose");
  factory.registerNodeType<erwinia_bt::LogTreeResult>("LogTreeResult");
  factory.registerNodeType<erwinia_bt::ShutdownSystem>("ShutdownSystem");

  try
  {
    const std::string plugin_path =
      ament_index_cpp::get_package_prefix("erwinia_bt_nodes") + "/lib/liberwinia_bt_nodes.so";
    factory.registerFromPlugin(plugin_path);
    RCLCPP_INFO(node->get_logger(), "Loaded BT plugin: %s", plugin_path.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(node->get_logger(), "BT plugin load failed: %s", e.what());
    RCLCPP_WARN(node->get_logger(), "Falling back to dummy action nodes");
    factory.registerNodeType<erwinia_bt::NavigateToPose>("NavigateToPose");
    factory.registerNodeType<erwinia_bt::ErwiniaDetector>("ErwiniaDetector");
    factory.registerNodeType<erwinia_bt::MarkLocation>("MarkLocation");
  }

  const std::string tree_path =
    ament_index_cpp::get_package_share_directory("erwinia_bt") + "/config/behavior_tree.xml";
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  // Live visualization in Groot
  const int groot_pub_port = 1666;
  const int groot_srv_port = 1667;
  const unsigned max_msgs_per_second = 25;
  BT::PublisherZMQ publisher(tree, max_msgs_per_second, groot_pub_port, groot_srv_port);
  RCLCPP_INFO(node->get_logger(),
              "Groot2 ZMQ: address=127.0.0.1 pub_port=%d srv_port=%d",
              groot_pub_port, groot_srv_port);


  rclcpp::Rate rate(0.1);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

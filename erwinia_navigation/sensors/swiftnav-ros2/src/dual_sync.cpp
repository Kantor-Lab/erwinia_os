#include "rclcpp/rclcpp.hpp"
#include "swiftnav_ros2_driver/msg/baseline.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

using BaselineMsg = swiftnav_ros2_driver::msg::Baseline;
using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
using OdomMsg = nav_msgs::msg::Odometry;

class SyncedBaselineNode : public rclcpp::Node
{
public:
  SyncedBaselineNode()
  : Node("synced_baseline_node")
  {
    using namespace std::placeholders;

    sub1_.subscribe(this, "/sw_ns_1/baseline");
    sub2_.subscribe(this, "/sw_ns_2/baseline");
    sub3_.subscribe(this, "/sw_ns_1/twistwithcovariancestamped");  // Replace with your actual topic name
    sub4_.subscribe(this, "/husky_velocity_controller/odom");

    sync_ = std::make_shared<Sync>(SyncPolicy(100), sub1_, sub2_, sub3_, sub4_);
    sync_->registerCallback(std::bind(&SyncedBaselineNode::callback, this, _1, _2, _3, _4));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("baseline_odom", 10);

    RCLCPP_INFO(this->get_logger(), "SyncedBaselineNode initialized.");
  }

private:
  void callback(const BaselineMsg::ConstSharedPtr msg1, 
    const BaselineMsg::ConstSharedPtr msg2,
    const TwistMsg::ConstSharedPtr twist_msg,
    const OdomMsg::ConstSharedPtr husky_odom_msg)
  {
    // Extract ENU from msg1
    double x = msg1->baseline_e_m;
    double y = msg1->baseline_n_m;

    double vel_x = twist_msg->twist.twist.linear.y;
    double vel_y = twist_msg->twist.twist.linear.x;
    double vel_z = -twist_msg->twist.twist.linear.z;

    double twist_z = husky_odom_msg->twist.twist.angular.z;

    // Use heading from msg2 (in degrees), convert to radians
    double theta_deg = msg2->baseline_dir_deg;  // Negate if direction convention is NED
    double theta_rad = (90-theta_deg) * (M_PI / 180.0);

    RCLCPP_INFO(this->get_logger(), "[x, y, theta] = [%.4f, %.4f, %.4f rad (%.2f deg)]",
                x, y, theta_rad, 90-theta_deg);

    // Convert yaw (theta) to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_rad);
    q.normalize();

    // Prepare Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    // odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vel_x;
    odom_msg.twist.twist.linear.y = vel_y;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = twist_z;

    odom_pub_->publish(odom_msg);
  }

  // Subscriptions and sync
  message_filters::Subscriber<BaselineMsg> sub1_;
  message_filters::Subscriber<BaselineMsg> sub2_;
  message_filters::Subscriber<TwistMsg> sub3_;
  message_filters::Subscriber<OdomMsg> sub4_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<BaselineMsg, BaselineMsg, TwistMsg, OdomMsg>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;

  // Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncedBaselineNode>());
  rclcpp::shutdown();
  return 0;
}


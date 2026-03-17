#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace erwinia_os_moveit_servo
{

// Xbox mapping (typical on Linux joy_node; verify with `ros2 topic echo /joy`)
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,   // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,  // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
  D_PAD_X = 6,
  D_PAD_Y = 7
};

enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

static inline double deadband(double v, double db)
{
  return (std::abs(v) < db) ? 0.0 : v;
}

class XboxToServoPub : public rclcpp::Node
{
public:
  explicit XboxToServoPub(const rclcpp::NodeOptions& options)
  : Node("xbox_to_servo_pub", options)
  {
    joy_topic_   = declare_parameter<std::string>("joy_topic", "/joy");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");

    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "xarm/xarm6_base_link");
    ee_frame_id_   = declare_parameter<std::string>("ee_frame_id",   "xarm/xarm6_link_eef");
    // frame_to_publish_ = ee_frame_id_;
    frame_to_publish_ = base_frame_id_;

    // deadman (RB by default)
    enable_button_ = declare_parameter<int>("enable_button", RIGHT_BUMPER);

    // Scales applied BEFORE Servo scaling (Servo also applies scale.linear/rotational if command_in_type=unitless)
    lin_scale_ = declare_parameter<double>("lin_scale", 1.0);
    ang_scale_ = declare_parameter<double>("ang_scale", 1.0);

    // Deadbands
    stick_deadband_   = declare_parameter<double>("stick_deadband", 0.08);
    trigger_deadband_ = declare_parameter<double>("trigger_deadband", 0.06);

    // Trigger defaults (many drivers report 1.0 at rest)
    left_trigger_default_  = declare_parameter<double>("left_trigger_default",  1.0);
    right_trigger_default_ = declare_parameter<double>("right_trigger_default", 1.0);

    // QoS: VOLATILE to avoid "sticky" commands
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&XboxToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, qos);

    // Optional: start Servo via service
    servo_start_service_ = declare_parameter<std::string>("servo_start_service", "/servo_node/start_servo");
    const bool autostart = declare_parameter<bool>("autostart_servo", true);
    if (autostart)
    {
      servo_start_client_ = create_client<std_srvs::srv::Trigger>(servo_start_service_);
      if (servo_start_client_->wait_for_service(std::chrono::seconds(1)))
      {
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Servo start service not available: %s", servo_start_service_.c_str());
      }
    }

    RCLCPP_INFO(get_logger(),
      "XboxToServoPub ready. joy=%s twist=%s base=%s ee=%s deadman_button=%d",
      joy_topic_.c_str(), twist_topic_.c_str(),
      base_frame_id_.c_str(), ee_frame_id_.c_str(), enable_button_);
  }

private:
  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Deadman required
    if (enable_button_ >= 0)
    {
      if (static_cast<size_t>(enable_button_) >= msg->buttons.size() || msg->buttons[enable_button_] == 0)
      {
        // Optionally publish a zero twist when deadman released (helps stop quickly)
        publishZero();
        return;
      }
    }

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    fillTwistFromJoy(msg->axes, *twist_msg);

    twist_msg->header.frame_id = frame_to_publish_;
    twist_msg->header.stamp = now();
    twist_pub_->publish(std::move(twist_msg));
  }

  void publishZero()
  {
    auto z = std::make_unique<geometry_msgs::msg::TwistStamped>();
    z->header.frame_id = frame_to_publish_;
    z->header.stamp = now();
    // all zeros by default
    twist_pub_->publish(std::move(z));
  }

  void fillTwistFromJoy(const std::vector<float>& axes, geometry_msgs::msg::TwistStamped& twist)
  {
    auto ax = [&](int idx) -> double {
      return (idx >= 0 && static_cast<size_t>(idx) < axes.size()) ? static_cast<double>(axes[idx]) : 0.0;
    };

    /**
     * Mapping (from eef frame perspective):
     * Note: on joystick the +X axis is to the left, +Y is up
     * 
     * Left stick up    +Y -> -Y (up)
     * Left stick down  -Y -> +Y (down)
     * Left stick right -X -> +X (right)
     * Left stick left  +X -> -X (left)
     * 
     * Right stick up    +Y -> +Roll (rotate camera upwards)
     * Right stick down  -Y -> -Roll (rotate camera downwards)
     * Right stick right -X -> +Pitch (rotate camera right)
     * Right stick left  +X -> -Pitch (rotate camera left)
     * 
     * Right trigger -> +Z (forward)
     * Left trigger  -> -Z (backward)
     * 
     * D-pad right -> +Yaw (rotate right)
     * D-pad left  -> -Yaw (rotate left)
     */

    /**
     * Mapping (from xarm base link frame perspective):
     * Note: on joystick the +X axis is to the left, +Y is up
     * 
     * Left stick up    +Y -> +Z (up)
     * Left stick down  -Y -> -Z (down)
     * Left stick right -X -> -Y (right)
     * Left stick left  +X -> +Y (left)
     * 
     * Right stick up    +Y -> -Pitch (rotate camera upwards)
     * Right stick down  -Y -> +Pitch (rotate camera downwards)
     * Right stick right -X -> -Yaw (rotate camera right)
     * Right stick left  +X -> +Yaw (rotate camera left)
     * 
     * Right trigger -> +X (forward)
     * Left trigger  -> -X (backward)
     * 
     * D-pad right -> +Roll (rotate right)
     * D-pad left  -> -Roll (rotate left)
     */

    // ---- Left stick -> Left/right and forward/backward camera translation ----
    double lx = deadband(ax(LEFT_STICK_X), stick_deadband_);
    double ly = deadband(ax(LEFT_STICK_Y), stick_deadband_);
    // twist.twist.linear.x = lin_scale_ * -lx; // for eef frame perspective
    // twist.twist.linear.y = lin_scale_ * -ly; // for eef frame perspective
    twist.twist.linear.y = lin_scale_ * lx;  // for base link frame perspective
    twist.twist.linear.z = lin_scale_ * ly;  // for base link frame perspective

    // ---- Triggers -> Up/down camera translation ----
    double rt_raw = ax(RIGHT_TRIGGER);
    double lt_raw = ax(LEFT_TRIGGER);
    // if default=1, rt_raw=-1 => rt=1
    double rt = 0.5 * (right_trigger_default_ - rt_raw);
    double lt = 0.5 * (left_trigger_default_  - lt_raw);
    rt = std::clamp(rt, 0.0, 1.0);
    lt = std::clamp(lt, 0.0, 1.0);
    rt = deadband(rt, trigger_deadband_);
    lt = deadband(lt, trigger_deadband_);
    // twist.twist.linear.z = lin_scale_ * (rt - lt); // for eef frame perspective
    twist.twist.linear.x = lin_scale_ * (rt - lt);  // for base link frame perspective

    // ---- Right stick -> Up/down and left/right camera rotation ----
    double rx = deadband(ax(RIGHT_STICK_X), stick_deadband_);
    double ry = deadband(ax(RIGHT_STICK_Y), stick_deadband_);
    // twist.twist.angular.y = ang_scale_ * -rx; // for eef frame perspective
    // twist.twist.angular.x = ang_scale_ * ry;  // for eef frame perspective
    twist.twist.angular.z = ang_scale_ * rx;   // for base link frame perspective
    twist.twist.angular.y = ang_scale_ * -ry;  // for base link frame perspective

    // Adjust yaw with the D-pad
    double dx = deadband(ax(D_PAD_X), stick_deadband_);
    // twist.twist.angular.z = ang_scale_ * -dx; // for eef frame perspective
    twist.twist.angular.x = ang_scale_ * -dx;  // for base link frame perspective
  }

private:
  // params
  std::string joy_topic_;
  std::string twist_topic_;
  std::string base_frame_id_;
  std::string ee_frame_id_;
  std::string frame_to_publish_;
  std::string servo_start_service_;

  int enable_button_{ RIGHT_BUMPER };

  double lin_scale_{ 1.0 };
  double ang_scale_{ 1.0 };

  double stick_deadband_{ 0.08 };
  double trigger_deadband_{ 0.06 };
  double left_trigger_default_{ 1.0 };
  double right_trigger_default_{ 1.0 };

  // ros
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};

}  // namespace erwinia_os_moveit_servo

RCLCPP_COMPONENTS_REGISTER_NODE(erwinia_os_moveit_servo::XboxToServoPub)

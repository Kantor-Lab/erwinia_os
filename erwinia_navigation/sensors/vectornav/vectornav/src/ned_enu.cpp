#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

using std::placeholders::_1;

class ImuNedToEnuNode : public rclcpp::Node
{
public:
    ImuNedToEnuNode() : Node("imu_ned2enu"), debug_(false)
    {
        // Declare and get debug parameter
        this->declare_parameter("debug", true);
        this->get_parameter("debug", debug_);

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/vectornav/imu", 10, std::bind(&ImuNedToEnuNode::imu_callback, this, _1));

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/vectornav/imu_enu", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        //sensor_msgs::msg::Imu new_imu_msg;
	sensor_msgs::msg::Imu new_imu_msg = *imu_msg;
	
        new_imu_msg.header.stamp = imu_msg->header.stamp;
        new_imu_msg.header.frame_id = "vectornav_enu";
        new_imu_msg.angular_velocity.x = imu_msg->angular_velocity.y;
        new_imu_msg.angular_velocity.y = -imu_msg->angular_velocity.x;
        new_imu_msg.angular_velocity.z = imu_msg->angular_velocity.z;

        new_imu_msg.linear_acceleration.x = imu_msg->linear_acceleration.y;
        new_imu_msg.linear_acceleration.y = -imu_msg->linear_acceleration.x;
        new_imu_msg.linear_acceleration.z = imu_msg->linear_acceleration.z;

        // ENU orientation
        //new_imu_msg.orientation.x = imu_msg->orientation.y;
        //new_imu_msg.orientation.y = -imu_msg->orientation.x;
        //new_imu_msg.orientation.z = imu_msg->orientation.z;
        //new_imu_msg.orientation.w = -imu_msg->orientation.w;

        if (debug_)
        {
            tf2::Quaternion raw_q, conv_q;
            tf2::fromMsg(imu_msg->orientation, raw_q);
            double raw_roll, raw_pitch, raw_yaw;
            tf2::Matrix3x3(raw_q).getRPY(raw_roll, raw_pitch, raw_yaw);

            tf2::fromMsg(new_imu_msg.orientation, conv_q);
            double conv_roll, conv_pitch, conv_yaw;
            tf2::Matrix3x3(conv_q).getRPY(conv_roll, conv_pitch, conv_yaw);

            RCLCPP_INFO(this->get_logger(), "raw_roll: %.4f, raw_pitch: %.4f, raw_yaw: %.4f",
                        raw_roll, raw_pitch, raw_yaw);
            RCLCPP_INFO(this->get_logger(), "conv_roll: %.4f, conv_pitch: %.4f, conv_yaw: %.4f",
                        conv_roll, conv_pitch, conv_yaw);
        }

        imu_publisher_->publish(new_imu_msg);
    }

    bool debug_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNedToEnuNode>());
    rclcpp::shutdown();
    return 0;
}


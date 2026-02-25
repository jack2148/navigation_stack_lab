#include "robot_base/wheel_odometry.hpp"

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "robot_base/msg/encoder_ticks.hpp"

namespace robot_base
{

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode() : Node("wheel_odometry")
  {
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);           // [m]
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.30);   // [m]
    ticks_per_rev_ = declare_parameter<double>("ticks_per_rev", 4096.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    max_dt_ = declare_parameter<double>("max_dt", 0.2);  // [s] guard

    sub_encoders_ = create_subscription<robot_base::msg::EncoderTicks>(
      "/encoders_raw", rclcpp::QoS(50),
      std::bind(&WheelOdometryNode::onEncoders, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(),
      "WheelOdometryNode started. wheel_radius=%.4f m, wheel_separation=%.4f m, ticks_per_rev=%.1f",
      wheel_radius_, wheel_separation_, ticks_per_rev_);
  }

private:
  void onEncoders(const robot_base::msg::EncoderTicks::SharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;

    if (!initialized_) {
      last_stamp_ = stamp;
      last_left_ticks_ = msg->left_ticks;
      last_right_ticks_ = msg->right_ticks;
      initialized_ = true;
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > max_dt_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping odom update due to dt=%.6f s (max_dt=%.3f).", dt, max_dt_);
      // Update last values anyway to avoid huge jumps on the next tick.
      last_stamp_ = stamp;
      last_left_ticks_ = msg->left_ticks;
      last_right_ticks_ = msg->right_ticks;
      return;
    }

    const int64_t dl = msg->left_ticks - last_left_ticks_;
    const int64_t dr = msg->right_ticks - last_right_ticks_;

    const double dphi_l = ticksToRadians(dl, ticks_per_rev_);  // [rad]
    const double dphi_r = ticksToRadians(dr, ticks_per_rev_);  // [rad]

    const double ds_l = dphi_l * wheel_radius_;  // [m]
    const double ds_r = dphi_r * wheel_radius_;  // [m]

    const double ds = 0.5 * (ds_r + ds_l);                  // [m]
    const double dtheta = (ds_r - ds_l) / wheel_separation_; // [rad]

    // Integrate pose using midpoint method
    const double theta_mid = yaw_ + 0.5 * dtheta;
    x_ += ds * std::cos(theta_mid);
    y_ += ds * std::sin(theta_mid);
    yaw_ += dtheta;

    // Normalize yaw to [-pi, pi] (optional)
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    // Publish Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    const double v = ds / dt;        // [m/s]
    const double w = dtheta / dt;    // [rad/s]
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    // Basic covariance (tune later)
    for (int i = 0; i < 36; ++i) {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.05 * 0.05;   // x
    odom.pose.covariance[7] = 0.05 * 0.05;   // y
    odom.pose.covariance[35] = 0.10 * 0.10;  // yaw (rough)
    odom.twist.covariance[0] = 0.1 * 0.1;    // v
    odom.twist.covariance[35] = 0.2 * 0.2;   // w

    pub_odom_->publish(odom);

    // Publish TF: odom -> base_link
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(tf);
    }

    // Update state
    last_stamp_ = stamp;
    last_left_ticks_ = msg->left_ticks;
    last_right_ticks_ = msg->right_ticks;
  }

  // Parameters
  double wheel_radius_{0.05};
  double wheel_separation_{0.30};
  double ticks_per_rev_{4096.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  double max_dt_{0.2};

  // State
  bool initialized_{false};
  rclcpp::Time last_stamp_{0, RCL_ROS_TIME};
  int64_t last_left_ticks_{0};
  int64_t last_right_ticks_{0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Subscription<robot_base::msg::EncoderTicks>::SharedPtr sub_encoders_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace robot_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_base::WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scan_planner
{
// Republishes an input Odometry with its child frame changed to the robot base.
//
// On the real robot fast_lio's /fastlio/odom is the IMU ("body") pose in the
// gravity-aligned world frame "camera_init", but the planner needs the robot
// base ("base_link") pose in that same world frame. Rather than depending on
// fast_lio's *dynamic* camera_init->body TF (whose timing/timestamps cause
// extrapolation when looked up at the odom stamp), this node takes the
// camera_init->body pose straight from the odom message and composes it with
// the *static* body->base_link mounting calibration, cached once from TF:
//
//   T(camera_init -> base_link) = T(camera_init -> body, from odom) * T(body -> base_link, static)
//
// Twist is intentionally left at zero (fast_lio never populates twist either,
// so this matches the input the planner already receives).
class BaselinkOdomTransform : public rclcpp::Node
{
public:
  BaselinkOdomTransform() : Node("baselink_odom_transform")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/fastlio/odom");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/baselink_odom");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "camera_init");
    source_child_frame_ = declare_parameter<std::string>("source_child_frame", "body");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.05);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&BaselinkOdomTransform::odomCallback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "baselink_odom_transform ready: %s (%s -> %s -> %s) -> %s",
                input_odom_topic_.c_str(), parent_frame_.c_str(), source_child_frame_.c_str(),
                child_frame_.c_str(), output_odom_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    // Cache the static source->child (body->base_link) calibration once.
    if (!have_static_)
    {
      try
      {
        const geometry_msgs::msg::TransformStamped stf = tf_buffer_->lookupTransform(
            source_child_frame_, child_frame_, tf2::TimePointZero,
            tf2::durationFromSec(transform_timeout_));
        static_t_ = Eigen::Vector3d(stf.transform.translation.x, stf.transform.translation.y,
                                    stf.transform.translation.z);
        static_q_ = Eigen::Quaterniond(stf.transform.rotation.w, stf.transform.rotation.x,
                                       stf.transform.rotation.y, stf.transform.rotation.z)
                        .normalized();
        have_static_ = true;
        RCLCPP_INFO(get_logger(), "Cached static %s->%s transform", source_child_frame_.c_str(),
                    child_frame_.c_str());
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Static %s->%s not yet available: %s", source_child_frame_.c_str(),
                             child_frame_.c_str(), ex.what());
        return;
      }
    }

    // T(parent -> source), i.e. camera_init -> body, taken directly from the odom message.
    const Eigen::Vector3d t_ps(msg->pose.pose.position.x, msg->pose.pose.position.y,
                               msg->pose.pose.position.z);
    const Eigen::Quaterniond q_ps(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // Compose: T(parent -> child) = T(parent -> source) * T(source -> child).
    const Eigen::Vector3d out_t = t_ps + q_ps * static_t_;
    const Eigen::Quaterniond out_q = (q_ps * static_q_).normalized();

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = parent_frame_;
    out.child_frame_id = child_frame_;
    out.pose.pose.position.x = out_t.x();
    out.pose.pose.position.y = out_t.y();
    out.pose.pose.position.z = out_t.z();
    out.pose.pose.orientation.x = out_q.x();
    out.pose.pose.orientation.y = out_q.y();
    out.pose.pose.orientation.z = out_q.z();
    out.pose.pose.orientation.w = out_q.w();
    // twist intentionally left zero: fast_lio never populates it, so this
    // matches the input the planner already receives.
    odom_pub_->publish(out);
  }

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string parent_frame_;
  std::string source_child_frame_;
  std::string child_frame_;
  double transform_timeout_{0.05};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool have_static_{false};
  Eigen::Vector3d static_t_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond static_q_{Eigen::Quaterniond::Identity()};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::BaselinkOdomTransform>());
  rclcpp::shutdown();
  return 0;
}

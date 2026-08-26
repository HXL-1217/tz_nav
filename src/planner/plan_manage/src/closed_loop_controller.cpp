#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    holonomic_ = declare_parameter<bool>("holonomic", true);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready (holonomic=%s)",
                holonomic_ ? "true" : "false");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;
  // Below this |velocity| (m/s), the heading is taken from the stable look-ahead
  // chord instead of atan2(vel) — atan2 of a near-zero vector is noise.
  static constexpr double kMinVelForHeading = 0.1;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);

    // DEBUG: trajectory shape (start/end/init-velocity direction) to inspect
    // whether the planner produced a sane trajectory for this goal.
    const Eigen::Vector3d dbg_p0 = traj_[0].evaluateDeBoorT(0.0);
    const Eigen::Vector3d dbg_pN = traj_[0].evaluateDeBoorT(traj_duration_);
    const Eigen::Vector3d dbg_v0 = traj_[1].evaluateDeBoorT(0.0);
    RCLCPP_INFO(get_logger(),
                "New traj dur=%.2fs start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f) init_vel_yaw=%.3f",
                traj_duration_, dbg_p0.x(), dbg_p0.y(), dbg_p0.z(),
                dbg_pN.x(), dbg_pN.y(), dbg_pN.z(), std::atan2(dbg_v0.y(), dbg_v0.x()));
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;

    // Reference at the current execution time (heading alignment + spin-in-place gate)
    const double t_eval = std::min(exec_time_, traj_duration_);
    const Eigen::Vector3d pos_des_eval = traj_[0].evaluateDeBoorT(t_eval);
    const Eigen::Vector2d pos_error_eval(pos_des_eval.x() - odom_pos_.x(),
                                         pos_des_eval.y() - odom_pos_.y());

    // Trajectory complete: once the time has elapsed and we are within
    // finish_dist (XY) of the endpoint, latch completion so the robot stands
    // still. Without this the controller keeps position-holding the old goal
    // (dragging the robot back to it after teleop) and, if the heading is
    // misaligned, spins in place forever because the spin check below would
    // run before the stop condition. A new trajectory re-enables tracking
    // (bsplineCallback sets receive_traj_ = true).
    if (exec_time_ >= traj_duration_ && pos_error_eval.norm() < finish_dist_)
    {
      receive_traj_ = false;
      publishExecutionFrozen(false);
      publishStop();
      RCLCPP_INFO(get_logger(), "Trajectory completed; holding position");
      return;
    }

    // Desired heading (mode-dependent). vel_world_eval is computed once here so
    // it is available for the non-holonomic yaw_des and for the diagnostics log.
    const Eigen::Vector3d vel_des_eval = traj_[1].evaluateDeBoorT(t_eval);
    const Eigen::Vector2d vel_world_eval = clampNorm(
        Eigen::Vector2d(vel_des_eval.x(), vel_des_eval.y()) + kp_pos_ * pos_error_eval, max_vx_);
    double yaw_des;
    if (holonomic_)
    {
      yaw_des = estimateDesiredYaw(t_eval, pos_des_eval);
    }
    else
    {
      // Align heading with the feedforward+feedback velocity when it is
      // meaningful; when it is near-zero (e.g. at the trajectory start, where
      // both vel_des and pos_error are ~0) fall back to the stable look-ahead
      // chord. Otherwise atan2 of a near-zero vector is noise and the desired
      // heading flips every cycle -> the robot spins forever on behind-goals.
      yaw_des = vel_world_eval.norm() > kMinVelForHeading
                    ? std::atan2(vel_world_eval.y(), vel_world_eval.x())
                    : estimateDesiredYaw(t_eval, pos_des_eval);
    }
    const double yaw_error = normalizeAngle(yaw_des - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);

    // Spin in place (and freeze trajectory progress) when badly misaligned
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;

    // Reference at the advanced execution time (drive command)
    const Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);

    geometry_msgs::msg::Twist command;
    if (holonomic_)
    {
      // Holonomic (e.g. Go2): full body-frame velocity, strafing allowed
      const Eigen::Vector2d vel_world = clampNorm(
          Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
          std::max(max_vx_, max_vy_));
      command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
      command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    }
    else
    {
      // Non-holonomic (双轮腿 / differential): forward along heading only, no strafe, forward-only
      const Eigen::Vector2d vel_world = clampNorm(
          Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error, max_vx_);
      const double vx_body = c * vel_world.x() + s * vel_world.y();
      command.linear.x = std::clamp(std::max(vx_body, 0.0), 0.0, max_vx_);
      command.linear.y = 0.0;
    }
    command.angular.z = yaw_command;
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  bool holonomic_{true};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}

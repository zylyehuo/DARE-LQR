#include "lqr/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

#include "robot_pose_transform.hpp"

namespace lqr
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

struct RobotPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SegmentProjection
{
  bool valid{false};
  double lateral_error{0.0};
  double heading_error{0.0};
  double distance_sq{std::numeric_limits<double>::max()};
  double progress{0.0};
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngleLocal(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

RobotPose2D robotBasePoseFromOdometry(
  const nav_msgs::msg::Odometry & odom,
  const nav_core::RobotBaseExtrinsics & extrinsics)
{
  const geometry_msgs::msg::Pose base_pose =
    nav_core::basePoseFromOdometry3D(odom, extrinsics);

  RobotPose2D pose;
  pose.x = base_pose.position.x;
  pose.y = base_pose.position.y;
  pose.yaw = normalizeAngleLocal(yawFromQuaternion(base_pose.orientation));
  return pose;
}

bool navCoreParkingAdjustmentEnabled(const rclcpp::Node::SharedPtr & node)
{
  bool enabled = true;
  if (node && node->has_parameter("parking_adjustment_enable")) {
    node->get_parameter("parking_adjustment_enable", enabled);
  }
  return enabled;
}

SegmentProjection computeNearestSegmentErrorInMotionFrame(
  const nav_msgs::msg::Path & path)
{
  SegmentProjection result;
  if (path.poses.size() < 2U) {
    return result;
  }

  constexpr double robot_x = 0.0;
  constexpr double robot_y = 0.0;
  double cumulative_length = 0.0;

  for (std::size_t index = 0; index + 1U < path.poses.size(); ++index) {
    const auto & from = path.poses[index].pose.position;
    const auto & to = path.poses[index + 1U].pose.position;

    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double segment_length_sq = dx * dx + dy * dy;
    if (segment_length_sq < 1.0e-12) {
      continue;
    }

    const double segment_length = std::sqrt(segment_length_sq);
    const double projection_ratio = std::max(
      0.0,
      std::min(
        1.0,
        ((robot_x - from.x) * dx + (robot_y - from.y) * dy) /
        segment_length_sq));

    const double projection_x = from.x + projection_ratio * dx;
    const double projection_y = from.y + projection_ratio * dy;
    const double error_x = robot_x - projection_x;
    const double error_y = robot_y - projection_y;
    const double distance_sq = error_x * error_x + error_y * error_y;

    if (distance_sq < result.distance_sq) {
      const double path_heading = std::atan2(dy, dx);
      const double cross =
        dx * (robot_y - from.y) - dy * (robot_x - from.x);

      double signed_lateral_error = std::sqrt(distance_sq);
      // Positive means the reference path is to the robot's left in the motion frame.
      if (cross > 0.0) {
        signed_lateral_error = -signed_lateral_error;
      }

      result.valid = true;
      result.distance_sq = distance_sq;
      result.progress = cumulative_length + projection_ratio * segment_length;
      result.lateral_error = signed_lateral_error;
      result.heading_error = normalizeAngleLocal(path_heading);
    }

    cumulative_length += segment_length;
  }

  return result;
}

}  // namespace

template<typename T>
T LqrController::declareParameter(
  const std::string & name,
  const T & default_value)
{
  if (!node_->has_parameter(name)) {
    return node_->declare_parameter<T>(name, default_value);
  }

  T value{};
  node_->get_parameter(name, value);
  return value;
}

double LqrController::clamp(
  double value,
  double min_value,
  double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double LqrController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void LqrController::initialize(
  const rclcpp::Node::SharedPtr & node,
  const std::string & plugin_name,
  tf2_ros::Buffer * tf_buffer,
  const nav_core::RobotBaseConfig & robot_base)
{
  (void)tf_buffer;

  node_ = node;
  name_ = plugin_name;
  robot_base_ = robot_base;

  target_linear_velocity_ =
    declareParameter("target_linear_velocity", 1.00);
  min_linear_velocity_ =
    declareParameter("min_linear_velocity", 0.06);
  max_linear_velocity_ =
    declareParameter("max_linear_velocity", 1.00);
  max_angular_velocity_ = declareParameter(
    "max_angular_velocity",
    robot_base.limits.max_angular_velocity);
  max_linear_acceleration_ =
    declareParameter("max_linear_acceleration", 0.80);

  wheel_base_ = declareParameter(
    "wheel_base",
    robot_base.footprint.wheel_base > 1.0e-6 ?
    robot_base.footprint.wheel_base : 0.494);
  track_width_ = declareParameter("track_width", 0.364);
  max_steering_angle_ = declareParameter("max_steering_angle", 0.698);
  min_turning_radius_ = declareParameter("min_turning_radius", 0.59);
  dual_ackermann_mode_radius_margin_ =
    declareParameter("dual_ackermann_mode_radius_margin", 0.03);

  lqr_q_lateral_ = declareParameter("lqr_q_lateral", 12.0);
  lqr_q_lateral_rate_ = declareParameter("lqr_q_lateral_rate", 0.30);
  lqr_q_heading_ = declareParameter("lqr_q_heading", 10.0);
  lqr_q_heading_rate_ = declareParameter("lqr_q_heading_rate", 0.25);
  lqr_r_steering_ = declareParameter("lqr_r_steering", 4.2);
  lqr_feedback_scale_ = declareParameter("lqr_feedback_scale", 1.00);
  lqr_error_rate_filter_alpha_ =
    declareParameter("lqr_error_rate_filter_alpha", 0.25);

  straight_reference_curvature_threshold_ =
    declareParameter("straight_reference_curvature_threshold", 0.025);
  straight_feedback_scale_ =
    declareParameter("straight_feedback_scale", 0.55);
  curve_feedback_scale_ =
    declareParameter("curve_feedback_scale", 1.10);
  straight_feedback_max_central_steering_ =
    declareParameter("straight_feedback_max_central_steering", 0.085);

  lateral_error_limit_ = declareParameter("lateral_error_limit", 0.25);
  heading_error_limit_ = declareParameter("heading_error_limit", 0.70);
  lateral_error_deadband_ =
    declareParameter("lateral_error_deadband", 0.0015);
  heading_error_deadband_ =
    declareParameter("heading_error_deadband", 0.0020);

  path_curvature_feedforward_gain_ =
    declareParameter("path_curvature_feedforward_gain", 1.00);
  path_curvature_window_distance_ =
    declareParameter("path_curvature_window_distance", 0.18);
  curve_preview_enable_ = declareParameter("curve_preview_enable", true);
  curve_preview_min_distance_ =
    declareParameter("curve_preview_min_distance", 0.08);
  curve_preview_speed_gain_ =
    declareParameter("curve_preview_speed_gain", 0.25);
  curve_preview_max_distance_ =
    declareParameter("curve_preview_max_distance", 0.38);
  curve_preview_samples_ = declareParameter("curve_preview_samples", 7);
  curve_preview_blend_ = declareParameter("curve_preview_blend", 0.50);
  curve_preview_activation_curvature_ =
    declareParameter("curve_preview_activation_curvature", 0.030);

  curve_anticipation_enable_ =
    declareParameter("curve_anticipation_enable", true);
  curve_anticipation_min_distance_ =
    declareParameter("curve_anticipation_min_distance", 0.90);
  curve_anticipation_speed_gain_ =
    declareParameter("curve_anticipation_speed_gain", 1.50);
  curve_anticipation_max_distance_ =
    declareParameter("curve_anticipation_max_distance", 2.50);
  curve_anticipation_samples_ =
    declareParameter("curve_anticipation_samples", 20);
  curve_anticipation_curvature_threshold_ =
    declareParameter("curve_anticipation_curvature_threshold", 0.050);
  curve_anticipation_lateral_acceleration_ =
    declareParameter("curve_anticipation_lateral_acceleration", 0.12);
  curve_anticipation_min_speed_ =
    declareParameter("curve_anticipation_min_speed", 0.24);
  curve_anticipation_deceleration_ =
    declareParameter("curve_anticipation_deceleration", 0.65);
  curve_anticipation_safety_distance_ =
    declareParameter("curve_anticipation_safety_distance", 0.25);

  lookahead_min_distance_ =
    declareParameter("lookahead_min_distance", 0.30);
  lookahead_max_distance_ =
    declareParameter("lookahead_max_distance", 0.85);
  lookahead_speed_gain_ =
    declareParameter("lookahead_speed_gain", 0.42);
  direction_filter_margin_ =
    declareParameter("direction_filter_margin", 0.08);
  minimum_reference_length_ =
    declareParameter("minimum_reference_length", 0.40);
  terminal_heading_lookback_distance_ =
    declareParameter("terminal_heading_lookback_distance", 0.65);

  path_acquisition_speed_limit_enable_ =
    declareParameter("path_acquisition_speed_limit_enable", true);
  path_acquisition_enter_lateral_error_ =
    declareParameter("path_acquisition_enter_lateral_error", 0.080);
  path_acquisition_exit_lateral_error_ =
    declareParameter("path_acquisition_exit_lateral_error", 0.025);
  path_acquisition_full_lateral_error_ =
    declareParameter("path_acquisition_full_lateral_error", 0.350);
  path_acquisition_enter_heading_error_ =
    declareParameter("path_acquisition_enter_heading_error", 0.220);
  path_acquisition_exit_heading_error_ =
    declareParameter("path_acquisition_exit_heading_error", 0.080);
  path_acquisition_full_heading_error_ =
    declareParameter("path_acquisition_full_heading_error", 0.800);
  path_reacquisition_enter_lateral_error_ =
    declareParameter("path_reacquisition_enter_lateral_error", 0.250);
  path_reacquisition_enter_heading_error_ =
    declareParameter("path_reacquisition_enter_heading_error", 0.550);
  path_acquisition_min_speed_ =
    declareParameter("path_acquisition_min_speed", 0.120);
  path_acquisition_max_speed_ =
    declareParameter("path_acquisition_max_speed", 0.550);
  path_acquisition_stable_cycles_ =
    declareParameter("path_acquisition_stable_cycles", 3);

  curvature_speed_limit_enable_ =
    declareParameter("curvature_speed_limit_enable", true);
  max_lateral_acceleration_ =
    declareParameter("max_lateral_acceleration", 0.32);
  error_speed_limit_enable_ =
    declareParameter("error_speed_limit_enable", true);
  lateral_error_speed_gain_ =
    declareParameter("lateral_error_speed_gain", 0.55);
  heading_error_speed_gain_ =
    declareParameter("heading_error_speed_gain", 0.38);
  speed_curvature_lateral_error_gain_ =
    declareParameter("speed_curvature_lateral_error_gain", 0.36);
  speed_curvature_heading_error_gain_ =
    declareParameter("speed_curvature_heading_error_gain", 0.66);

  terminal_stop_on_path_end_enable_ =
    declareParameter("terminal_stop_on_path_end_enable", true);
  terminal_stop_latch_enable_ =
    declareParameter("terminal_stop_latch_enable", true);
  terminal_slowdown_enable_ =
    declareParameter("terminal_slowdown_enable", true);
  terminal_slowdown_distance_ =
    declareParameter("terminal_slowdown_distance", 2.00);
  terminal_stop_distance_ =
    declareParameter("terminal_stop_distance", 0.010);
  terminal_stop_exit_distance_ =
    declareParameter("terminal_stop_exit_distance", 0.030);
  terminal_stop_hold_cycles_ =
    declareParameter("terminal_stop_hold_cycles", 3);
  terminal_min_linear_velocity_ =
    declareParameter("terminal_min_linear_velocity", 0.010);
  terminal_longitudinal_speed_gain_ =
    declareParameter("terminal_longitudinal_speed_gain", 0.50);
  terminal_stop_s_tolerance_ =
    declareParameter("terminal_stop_s_tolerance", 0.010);
  terminal_stop_lateral_tolerance_ =
    declareParameter("terminal_stop_lateral_tolerance", 0.010);
  terminal_correction_min_velocity_ =
    declareParameter("terminal_correction_min_velocity", 0.010);
  terminal_correction_speed_max_ =
    declareParameter("terminal_correction_speed_max", 0.040);
  terminal_lateral_error_speed_floor_gain_ =
    declareParameter("terminal_lateral_error_speed_floor_gain", 1.80);
  terminal_distance_error_speed_floor_gain_ =
    declareParameter("terminal_distance_error_speed_floor_gain", 0.30);

  reverse_terminal_lateral_convergence_enable_ =
    declareParameter("reverse_terminal_lateral_convergence_enable", true);
  reverse_terminal_lateral_convergence_start_distance_ =
    declareParameter("reverse_terminal_lateral_convergence_start_distance", 1.50);
  reverse_terminal_lateral_convergence_fade_distance_ =
    declareParameter("reverse_terminal_lateral_convergence_fade_distance", 0.08);
  reverse_terminal_lateral_convergence_deadband_ =
    declareParameter("reverse_terminal_lateral_convergence_deadband", 0.004);
  reverse_terminal_lateral_convergence_gain_ =
    declareParameter("reverse_terminal_lateral_convergence_gain", 1.35);
  reverse_terminal_lateral_convergence_max_steering_ =
    declareParameter("reverse_terminal_lateral_convergence_max_steering", 0.035);

  curvature_smoothing_enable_ =
    declareParameter("curvature_smoothing_enable", true);
  curvature_smoothing_alpha_ =
    declareParameter("curvature_smoothing_alpha", 0.55);
  curve_entry_smoothing_alpha_ =
    declareParameter("curve_entry_smoothing_alpha", 0.85);
  curve_exit_smoothing_alpha_ =
    declareParameter("curve_exit_smoothing_alpha", 0.72);
  curve_entry_rate_multiplier_ =
    declareParameter("curve_entry_rate_multiplier", 1.40);
  max_curvature_rate_ = declareParameter("max_curvature_rate", 3.00);
  straight_sign_change_smoothing_alpha_ =
    declareParameter("straight_sign_change_smoothing_alpha", 0.30);
  straight_sign_change_rate_multiplier_ =
    declareParameter("straight_sign_change_rate_multiplier", 0.45);
  control_period_ = declareParameter("control_period", 0.10);

  command_smoothing_enable_ =
    declareParameter("command_smoothing_enable", true);
  command_smoothing_alpha_ =
    declareParameter("command_smoothing_alpha", 0.65);
  command_deceleration_alpha_ =
    declareParameter("command_deceleration_alpha", 1.00);

  target_linear_velocity_ = std::max(0.0, target_linear_velocity_);
  min_linear_velocity_ = std::max(0.0, min_linear_velocity_);
  max_linear_velocity_ = std::max(min_linear_velocity_, max_linear_velocity_);
  max_angular_velocity_ = std::max(0.0, max_angular_velocity_);
  max_linear_acceleration_ = std::max(0.0, max_linear_acceleration_);

  wheel_base_ = std::max(0.05, wheel_base_);
  track_width_ = std::max(0.0, track_width_);
  max_steering_angle_ = clamp(std::fabs(max_steering_angle_), 0.05, 1.20);
  min_turning_radius_ = std::max(0.05, min_turning_radius_);
  dual_ackermann_mode_radius_margin_ =
    std::max(0.0, dual_ackermann_mode_radius_margin_);

  const double dual_ackermann_inner_limit = std::atan(
    0.5 * wheel_base_ /
    (min_turning_radius_ + dual_ackermann_mode_radius_margin_));
  const double usable_inner_steering =
    std::min(max_steering_angle_, dual_ackermann_inner_limit);
  max_central_steering_angle_ =
    std::fabs(innerSteeringToCentral(usable_inner_steering));

  lqr_q_lateral_ = std::max(0.0, lqr_q_lateral_);
  lqr_q_lateral_rate_ = std::max(0.0, lqr_q_lateral_rate_);
  lqr_q_heading_ = std::max(0.0, lqr_q_heading_);
  lqr_q_heading_rate_ = std::max(0.0, lqr_q_heading_rate_);
  lqr_r_steering_ = std::max(1.0e-6, lqr_r_steering_);
  lqr_feedback_scale_ = std::max(0.0, lqr_feedback_scale_);
  lqr_error_rate_filter_alpha_ =
    clamp(lqr_error_rate_filter_alpha_, 0.0, 1.0);

  straight_reference_curvature_threshold_ =
    std::max(0.0, straight_reference_curvature_threshold_);
  straight_feedback_scale_ = std::max(0.0, straight_feedback_scale_);
  curve_feedback_scale_ = std::max(0.0, curve_feedback_scale_);
  straight_feedback_max_central_steering_ = clamp(
    std::fabs(straight_feedback_max_central_steering_),
    0.01,
    max_central_steering_angle_);

  lateral_error_limit_ = std::max(0.0, lateral_error_limit_);
  heading_error_limit_ = std::max(0.0, heading_error_limit_);
  lateral_error_deadband_ = std::max(0.0, lateral_error_deadband_);
  heading_error_deadband_ = std::max(0.0, heading_error_deadband_);

  path_curvature_window_distance_ =
    std::max(0.05, path_curvature_window_distance_);
  curve_preview_min_distance_ =
    std::max(0.0, curve_preview_min_distance_);
  curve_preview_max_distance_ =
    std::max(curve_preview_min_distance_, curve_preview_max_distance_);
  curve_preview_speed_gain_ = std::max(0.0, curve_preview_speed_gain_);
  curve_preview_samples_ = std::max(1, curve_preview_samples_);
  curve_preview_blend_ = clamp(curve_preview_blend_, 0.0, 1.0);
  curve_preview_activation_curvature_ =
    std::max(0.0, curve_preview_activation_curvature_);

  curve_anticipation_min_distance_ =
    std::max(0.0, curve_anticipation_min_distance_);
  curve_anticipation_max_distance_ = std::max(
    curve_anticipation_min_distance_,
    curve_anticipation_max_distance_);
  curve_anticipation_speed_gain_ =
    std::max(0.0, curve_anticipation_speed_gain_);
  curve_anticipation_samples_ = std::max(1, curve_anticipation_samples_);
  curve_anticipation_curvature_threshold_ =
    std::max(0.0, curve_anticipation_curvature_threshold_);
  curve_anticipation_lateral_acceleration_ =
    std::max(0.01, curve_anticipation_lateral_acceleration_);
  curve_anticipation_min_speed_ = clamp(
    curve_anticipation_min_speed_,
    0.0,
    max_linear_velocity_);
  curve_anticipation_deceleration_ =
    std::max(0.01, curve_anticipation_deceleration_);
  curve_anticipation_safety_distance_ =
    std::max(0.0, curve_anticipation_safety_distance_);

  lookahead_min_distance_ = std::max(0.05, lookahead_min_distance_);
  lookahead_max_distance_ =
    std::max(lookahead_min_distance_, lookahead_max_distance_);
  lookahead_speed_gain_ = std::max(0.0, lookahead_speed_gain_);
  direction_filter_margin_ = std::max(0.0, direction_filter_margin_);
  minimum_reference_length_ = std::max(0.0, minimum_reference_length_);
  terminal_heading_lookback_distance_ =
    std::max(0.05, terminal_heading_lookback_distance_);

  path_acquisition_enter_lateral_error_ =
    std::max(0.0, path_acquisition_enter_lateral_error_);
  path_acquisition_exit_lateral_error_ = clamp(
    path_acquisition_exit_lateral_error_,
    0.0,
    path_acquisition_enter_lateral_error_);
  path_acquisition_full_lateral_error_ = std::max(
    path_acquisition_enter_lateral_error_ + 1.0e-6,
    path_acquisition_full_lateral_error_);
  path_acquisition_enter_heading_error_ =
    std::max(0.0, path_acquisition_enter_heading_error_);
  path_acquisition_exit_heading_error_ = clamp(
    path_acquisition_exit_heading_error_,
    0.0,
    path_acquisition_enter_heading_error_);
  path_acquisition_full_heading_error_ = std::max(
    path_acquisition_enter_heading_error_ + 1.0e-6,
    path_acquisition_full_heading_error_);
  path_reacquisition_enter_lateral_error_ = std::max(
    path_acquisition_enter_lateral_error_,
    path_reacquisition_enter_lateral_error_);
  path_reacquisition_enter_heading_error_ = std::max(
    path_acquisition_enter_heading_error_,
    path_reacquisition_enter_heading_error_);
  path_acquisition_min_speed_ = clamp(
    path_acquisition_min_speed_,
    0.0,
    max_linear_velocity_);
  path_acquisition_max_speed_ = clamp(
    path_acquisition_max_speed_,
    path_acquisition_min_speed_,
    max_linear_velocity_);
  path_acquisition_stable_cycles_ =
    std::max(1, path_acquisition_stable_cycles_);

  max_lateral_acceleration_ = std::max(0.01, max_lateral_acceleration_);
  lateral_error_speed_gain_ = std::max(0.0, lateral_error_speed_gain_);
  heading_error_speed_gain_ = std::max(0.0, heading_error_speed_gain_);
  speed_curvature_lateral_error_gain_ =
    std::max(0.0, speed_curvature_lateral_error_gain_);
  speed_curvature_heading_error_gain_ =
    std::max(0.0, speed_curvature_heading_error_gain_);

  terminal_slowdown_distance_ =
    std::max(terminal_stop_distance_ + 1.0e-3, terminal_slowdown_distance_);
  terminal_stop_distance_ = std::max(0.001, terminal_stop_distance_);
  terminal_stop_exit_distance_ =
    std::max(terminal_stop_distance_, terminal_stop_exit_distance_);
  terminal_stop_hold_cycles_ = std::max(1, terminal_stop_hold_cycles_);
  terminal_min_linear_velocity_ = clamp(
    terminal_min_linear_velocity_,
    0.0,
    max_linear_velocity_);
  terminal_longitudinal_speed_gain_ =
    std::max(0.0, terminal_longitudinal_speed_gain_);
  terminal_stop_s_tolerance_ = std::max(0.0, terminal_stop_s_tolerance_);
  terminal_stop_lateral_tolerance_ =
    std::max(0.0, terminal_stop_lateral_tolerance_);
  terminal_correction_min_velocity_ = clamp(
    terminal_correction_min_velocity_,
    terminal_min_linear_velocity_,
    max_linear_velocity_);
  terminal_correction_speed_max_ = clamp(
    terminal_correction_speed_max_,
    terminal_correction_min_velocity_,
    max_linear_velocity_);
  terminal_lateral_error_speed_floor_gain_ =
    std::max(0.0, terminal_lateral_error_speed_floor_gain_);
  terminal_distance_error_speed_floor_gain_ =
    std::max(0.0, terminal_distance_error_speed_floor_gain_);
  reverse_terminal_lateral_convergence_start_distance_ = std::max(
    terminal_stop_distance_,
    reverse_terminal_lateral_convergence_start_distance_);
  reverse_terminal_lateral_convergence_fade_distance_ = clamp(
    reverse_terminal_lateral_convergence_fade_distance_,
    terminal_stop_distance_,
    reverse_terminal_lateral_convergence_start_distance_);
  reverse_terminal_lateral_convergence_deadband_ =
    std::max(0.0, reverse_terminal_lateral_convergence_deadband_);
  reverse_terminal_lateral_convergence_gain_ =
    std::max(0.0, reverse_terminal_lateral_convergence_gain_);
  reverse_terminal_lateral_convergence_max_steering_ = clamp(
    std::fabs(reverse_terminal_lateral_convergence_max_steering_),
    0.0,
    max_central_steering_angle_);

  curvature_smoothing_alpha_ = clamp(curvature_smoothing_alpha_, 0.0, 1.0);
  curve_entry_smoothing_alpha_ = clamp(
    curve_entry_smoothing_alpha_,
    curvature_smoothing_alpha_,
    1.0);
  curve_exit_smoothing_alpha_ = clamp(
    curve_exit_smoothing_alpha_,
    curvature_smoothing_alpha_,
    1.0);
  curve_entry_rate_multiplier_ =
    std::max(1.0, curve_entry_rate_multiplier_);
  max_curvature_rate_ = std::max(0.05, max_curvature_rate_);
  straight_sign_change_smoothing_alpha_ =
    clamp(straight_sign_change_smoothing_alpha_, 0.0, 1.0);
  straight_sign_change_rate_multiplier_ =
    clamp(straight_sign_change_rate_multiplier_, 0.05, 1.0);
  control_period_ = std::max(0.01, control_period_);

  command_smoothing_alpha_ = clamp(command_smoothing_alpha_, 0.0, 1.0);
  command_deceleration_alpha_ = clamp(command_deceleration_alpha_, 0.0, 1.0);

  const double steering_curvature_limit =
    std::fabs(centralSteeringToCurvature(max_central_steering_angle_));
  const double radius_curvature_limit = 1.0 / min_turning_radius_;
  max_curvature_ = std::max(
    1.0e-6,
    std::min(steering_curvature_limit, radius_curvature_limit));

  lookahead_point_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/lookahead_point",
    1);
  tracking_error_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/tracking_error",
    10);

  reset();
  initialized_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] Clean dual-Ackermann DARE-LQR initialized: "
    "Q=[%.2f, %.2f, %.2f, %.2f], R=%.2f, dt=%.3f, "
    "max_v=%.2f, wheel_base=%.3f, max_curvature=%.3f, stop=%.3f.",
    name_.c_str(),
    lqr_q_lateral_,
    lqr_q_lateral_rate_,
    lqr_q_heading_,
    lqr_q_heading_rate_,
    lqr_r_steering_,
    control_period_,
    max_linear_velocity_,
    wheel_base_,
    max_curvature_,
    terminal_stop_distance_);
}

void LqrController::reset()
{
  has_last_command_ = false;
  last_cmd_ = geometry_msgs::msg::Twist{};

  has_last_curvature_ = false;
  last_curvature_ = 0.0;
  last_direction_ = 0;

  has_error_history_ = false;
  last_lateral_error_ = 0.0;
  last_heading_error_ = 0.0;
  filtered_lateral_error_rate_ = 0.0;
  filtered_heading_error_rate_ = 0.0;

  path_acquisition_active_ = false;
  path_acquired_once_ = false;
  path_acquisition_stable_count_ = 0;

  terminal_hold_count_ = 0;
  terminal_stop_latched_ = false;
  terminal_stop_result_printed_ = false;
  control_debug_ = ControlDebug{};
}

double LqrController::innerSteeringToCentral(double inner_steering) const
{
  const double magnitude = std::fabs(inner_steering);
  const double numerator = wheel_base_ * std::sin(magnitude);
  const double denominator =
    wheel_base_ * std::cos(magnitude) +
    track_width_ * std::sin(magnitude);
  const double central =
    std::atan2(numerator, std::max(denominator, 1.0e-9));
  return std::copysign(central, inner_steering);
}

double LqrController::centralSteeringToInner(double central_steering) const
{
  const double limited = clamp(
    central_steering,
    -max_central_steering_angle_,
    max_central_steering_angle_);
  const double magnitude = std::fabs(limited);
  const double numerator = wheel_base_ * std::sin(magnitude);
  const double denominator =
    wheel_base_ * std::cos(magnitude) -
    track_width_ * std::sin(magnitude);
  const double inner =
    std::atan2(numerator, std::max(denominator, 1.0e-9));
  return std::copysign(inner, limited);
}

double LqrController::curvatureToCentralSteering(double curvature) const
{
  const double central = std::atan(0.5 * wheel_base_ * curvature);
  return clamp(
    central,
    -max_central_steering_angle_,
    max_central_steering_angle_);
}

double LqrController::centralSteeringToCurvature(double central_steering) const
{
  const double limited = clamp(
    central_steering,
    -max_central_steering_angle_,
    max_central_steering_angle_);
  return 2.0 * std::tan(limited) / std::max(wheel_base_, 1.0e-6);
}

double LqrController::twistYawRateFromCurvature(
  double speed_abs,
  double curvature) const
{
  if (speed_abs <= 1.0e-9) {
    return 0.0;
  }

  const double central = curvatureToCentralSteering(curvature);
  const double inner = clamp(
    centralSteeringToInner(central),
    -max_steering_angle_,
    max_steering_angle_);
  const double yaw_rate =
    2.0 * speed_abs * std::tan(inner) / std::max(wheel_base_, 1.0e-6);
  return clamp(yaw_rate, -max_angular_velocity_, max_angular_velocity_);
}

bool LqrController::buildFallbackPath(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::PoseStamped & tracking_point,
  nav_msgs::msg::Path & effective_path) const
{
  const RobotPose2D robot_pose =
    robotBasePoseFromOdometry(odom, robot_base_.extrinsics);

  const double quaternion_norm = std::sqrt(
    tracking_point.pose.orientation.x * tracking_point.pose.orientation.x +
    tracking_point.pose.orientation.y * tracking_point.pose.orientation.y +
    tracking_point.pose.orientation.z * tracking_point.pose.orientation.z +
    tracking_point.pose.orientation.w * tracking_point.pose.orientation.w);
  if (!std::isfinite(quaternion_norm) || quaternion_norm <= 1.0e-6) {
    return false;
  }

  const double dx = tracking_point.pose.position.x - robot_pose.x;
  const double dy = tracking_point.pose.position.y - robot_pose.y;
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);

  const double terminal_x = cosine * dx + sine * dy;
  const double terminal_y = -sine * dx + cosine * dy;
  const double terminal_heading = normalizeAngle(
    yawFromQuaternion(tracking_point.pose.orientation) - robot_pose.yaw);

  if (!std::isfinite(terminal_x) || !std::isfinite(terminal_y) ||
    !std::isfinite(terminal_heading))
  {
    return false;
  }

  effective_path = nav_msgs::msg::Path{};
  effective_path.header.frame_id =
    robot_base_.base_frame.empty() ? "base_link" : robot_base_.base_frame;
  effective_path.header.stamp = odom.header.stamp;

  constexpr double kBehindLength = 0.60;
  constexpr double kAheadLength = 0.80;
  constexpr double kSpacing = 0.10;
  const double tangent_x = std::cos(terminal_heading);
  const double tangent_y = std::sin(terminal_heading);

  auto append_pose = [&](double offset) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = effective_path.header;
      pose.pose.position.x = terminal_x + offset * tangent_x;
      pose.pose.position.y = terminal_y + offset * tangent_y;
      pose.pose.orientation.z = std::sin(0.5 * terminal_heading);
      pose.pose.orientation.w = std::cos(0.5 * terminal_heading);
      effective_path.poses.push_back(pose);
    };

  for (double offset = -kBehindLength;
    offset < kAheadLength - 1.0e-9;
    offset += kSpacing)
  {
    append_pose(offset);
  }
  append_pose(kAheadLength);

  return effective_path.poses.size() >= 2U;
}

bool LqrController::transformPathToMotionFrame(
  const nav_msgs::msg::Path & local_path,
  int driving_direction_sign,
  nav_msgs::msg::Path & motion_path) const
{
  motion_path = local_path;
  motion_path.poses.clear();
  if (local_path.poses.empty()) {
    return false;
  }

  const int direction = driving_direction_sign >= 0 ? 1 : -1;
  motion_path.poses.reserve(local_path.poses.size());
  for (const auto & pose : local_path.poses) {
    geometry_msgs::msg::PoseStamped transformed = pose;
    transformed.pose.position.x =
      static_cast<double>(direction) * pose.pose.position.x;
    transformed.pose.position.y =
      static_cast<double>(direction) * pose.pose.position.y;
    motion_path.poses.push_back(transformed);
  }

  return !motion_path.poses.empty();
}

bool LqrController::filterMotionPath(
  const nav_msgs::msg::Path & motion_path,
  nav_msgs::msg::Path & filtered_path) const
{
  filtered_path = motion_path;
  filtered_path.poses.clear();
  filtered_path.poses.reserve(motion_path.poses.size());

  for (const auto & pose : motion_path.poses) {
    if (pose.pose.position.x >= -direction_filter_margin_) {
      filtered_path.poses.push_back(pose);
    }
  }

  if (filtered_path.poses.size() < 2U) {
    filtered_path = motion_path;
  }
  return filtered_path.poses.size() >= 2U;
}

double LqrController::pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

double LqrController::terminalHeadingFromMotionPath(
  const nav_msgs::msg::Path & motion_path) const
{
  if (motion_path.poses.size() < 2U) {
    return 0.0;
  }

  const auto & terminal = motion_path.poses.back().pose.position;
  double accumulated = 0.0;
  std::size_t lookback_index = motion_path.poses.size() - 2U;

  for (std::size_t index = motion_path.poses.size() - 1U;
    index > 0U;
    --index)
  {
    const auto & current = motion_path.poses[index].pose.position;
    const auto & previous = motion_path.poses[index - 1U].pose.position;
    accumulated += std::hypot(
      current.x - previous.x,
      current.y - previous.y);
    lookback_index = index - 1U;
    if (accumulated >= terminal_heading_lookback_distance_) {
      break;
    }
  }

  const auto & previous = motion_path.poses[lookback_index].pose.position;
  const double dx = terminal.x - previous.x;
  const double dy = terminal.y - previous.y;
  if (std::hypot(dx, dy) < 1.0e-6) {
    return 0.0;
  }
  return normalizeAngle(std::atan2(dy, dx));
}

double LqrController::pathCurvatureAtProgress(
  const nav_msgs::msg::Path & path,
  double progress,
  double window_distance) const
{
  if (path.poses.size() < 3U) {
    return 0.0;
  }

  const double total_length = pathLength(path);
  if (total_length < 0.08) {
    return 0.0;
  }

  auto point_at_arc_length = [&](double target_s, double & x, double & y) {
      target_s = clamp(target_s, 0.0, total_length);
      double accumulated = 0.0;

      for (std::size_t index = 1U; index < path.poses.size(); ++index) {
        const auto & previous = path.poses[index - 1U].pose.position;
        const auto & current = path.poses[index].pose.position;
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        const double segment_length = std::hypot(dx, dy);
        if (segment_length < 1.0e-9) {
          continue;
        }

        if (accumulated + segment_length >= target_s) {
          const double ratio = clamp(
            (target_s - accumulated) / segment_length,
            0.0,
            1.0);
          x = previous.x + ratio * dx;
          y = previous.y + ratio * dy;
          return;
        }
        accumulated += segment_length;
      }

      x = path.poses.back().pose.position.x;
      y = path.poses.back().pose.position.y;
    };

  const double span = clamp(
    std::max(window_distance, 0.12),
    0.12,
    total_length);
  const double current_s = clamp(progress, 0.0, total_length);

  double s0 = 0.0;
  double s1 = 0.0;
  double s2 = 0.0;
  if (current_s < 0.5 * span) {
    s0 = current_s;
    s1 = clamp(current_s + 0.5 * span, 0.0, total_length);
    s2 = clamp(current_s + span, 0.0, total_length);
  } else if (current_s + 0.5 * span > total_length) {
    s2 = current_s;
    s1 = clamp(current_s - 0.5 * span, 0.0, total_length);
    s0 = clamp(current_s - span, 0.0, total_length);
  } else {
    s0 = current_s - 0.5 * span;
    s1 = current_s;
    s2 = current_s + 0.5 * span;
  }

  if ((s1 - s0) < 0.025 || (s2 - s1) < 0.025) {
    return 0.0;
  }

  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;
  point_at_arc_length(s0, x0, y0);
  point_at_arc_length(s1, x1, y1);
  point_at_arc_length(s2, x2, y2);

  const double side_a = std::hypot(x1 - x0, y1 - y0);
  const double side_b = std::hypot(x2 - x1, y2 - y1);
  const double side_c = std::hypot(x2 - x0, y2 - y0);
  const double denominator = side_a * side_b * side_c;
  if (denominator < 1.0e-8) {
    return 0.0;
  }

  const double cross =
    (x1 - x0) * (y2 - y0) -
    (y1 - y0) * (x2 - x0);
  const double curvature = 2.0 * cross / denominator;
  if (!std::isfinite(curvature)) {
    return 0.0;
  }

  return clamp(curvature, -max_curvature_, max_curvature_);
}

double LqrController::previewPathCurvatureAtProgress(
  const nav_msgs::msg::Path & path,
  double progress,
  double target_speed) const
{
  const double current_curvature = pathCurvatureAtProgress(
    path,
    progress,
    path_curvature_window_distance_);
  if (!curve_preview_enable_ || path.poses.size() < 3U) {
    return current_curvature;
  }

  const double preview_distance = clamp(
    curve_preview_min_distance_ +
    curve_preview_speed_gain_ * std::fabs(target_speed),
    curve_preview_min_distance_,
    curve_preview_max_distance_);
  if (preview_distance <= 1.0e-6) {
    return current_curvature;
  }

  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  for (int sample = 1; sample <= curve_preview_samples_; ++sample) {
    const double ratio =
      static_cast<double>(sample) /
      static_cast<double>(curve_preview_samples_);
    const double sample_curvature = pathCurvatureAtProgress(
      path,
      progress + ratio * preview_distance,
      path_curvature_window_distance_);
    const double weight = 1.0 - 0.45 * ratio;
    weighted_sum += weight * sample_curvature;
    weight_sum += weight;
  }

  const double preview_mean =
    weight_sum > 1.0e-9 ? weighted_sum / weight_sum : current_curvature;
  const double activation = std::max(
    std::fabs(current_curvature),
    std::fabs(preview_mean));
  if (activation <= curve_preview_activation_curvature_) {
    return current_curvature;
  }

  const double activation_scale = clamp(
    (activation - curve_preview_activation_curvature_) /
    std::max(2.0 * curve_preview_activation_curvature_, 0.05),
    0.0,
    1.0);
  const double blend = curve_preview_blend_ * activation_scale;
  return clamp(
    (1.0 - blend) * current_curvature + blend * preview_mean,
    -max_curvature_,
    max_curvature_);
}

double LqrController::computeCurveAnticipationSpeedLimit(
  const nav_msgs::msg::Path & path,
  double progress,
  double target_speed) const
{
  if (!curve_anticipation_enable_ || path.poses.size() < 3U) {
    return max_linear_velocity_;
  }

  const double total_length = pathLength(path);
  if (total_length <= progress + 1.0e-4) {
    return max_linear_velocity_;
  }

  const double preview_distance = clamp(
    curve_anticipation_min_distance_ +
    curve_anticipation_speed_gain_ * std::fabs(target_speed),
    curve_anticipation_min_distance_,
    curve_anticipation_max_distance_);
  const double available_horizon = std::min(
    preview_distance,
    std::max(0.0, total_length - progress));
  if (available_horizon <= 1.0e-4) {
    return max_linear_velocity_;
  }

  bool curve_found = false;
  double curve_entry_distance = available_horizon;
  double peak_abs_curvature = 0.0;

  for (int sample = 0; sample <= curve_anticipation_samples_; ++sample) {
    const double ratio =
      static_cast<double>(sample) /
      static_cast<double>(curve_anticipation_samples_);
    const double offset = ratio * available_horizon;
    const double abs_curvature = std::fabs(pathCurvatureAtProgress(
      path,
      progress + offset,
      path_curvature_window_distance_));

    if (abs_curvature >= curve_anticipation_curvature_threshold_) {
      if (!curve_found) {
        curve_found = true;
        curve_entry_distance = offset;
      }
      peak_abs_curvature = std::max(peak_abs_curvature, abs_curvature);
    }
  }

  if (!curve_found || peak_abs_curvature < 1.0e-6) {
    return max_linear_velocity_;
  }

  const double curve_speed = clamp(
    std::sqrt(
      curve_anticipation_lateral_acceleration_ /
      std::max(peak_abs_curvature, 1.0e-6)),
    curve_anticipation_min_speed_,
    max_linear_velocity_);
  const double braking_distance = std::max(
    0.0,
    curve_entry_distance - curve_anticipation_safety_distance_);
  const double allowed_speed = std::sqrt(std::max(
    0.0,
    curve_speed * curve_speed +
    2.0 * curve_anticipation_deceleration_ * braking_distance));

  return clamp(
    allowed_speed,
    curve_anticipation_min_speed_,
    max_linear_velocity_);
}

LqrController::TerminalTarget LqrController::computeExternalTerminalTarget(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::PoseStamped & terminal_point,
  int driving_direction_sign) const
{
  TerminalTarget target;
  // odom.pose is the localization-sensor pose. Convert it exactly once to
  // current base_link. nav_core supplies terminal_point as the desired
  // base_link endpoint in map. Therefore every terminal error below is
  // target base_link minus current base_link.
  const RobotPose2D robot_pose =
    robotBasePoseFromOdometry(odom, robot_base_.extrinsics);

  const double dx = terminal_point.pose.position.x - robot_pose.x;
  const double dy = terminal_point.pose.position.y - robot_pose.y;
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  const double x_base = cosine * dx + sine * dy;
  const double y_base = -sine * dx + cosine * dy;
  const int direction = driving_direction_sign >= 0 ? 1 : -1;

  target.x = static_cast<double>(direction) * x_base;
  target.y = static_cast<double>(direction) * y_base;
  target.distance = std::hypot(target.x, target.y);

  // nav_core supplies the terminal quaternion as the executed path travel tangent.
  // For reverse motion, rotating into the motion frame removes the same pi offset.
  const double endpoint_heading_error_base = normalizeAngle(
    yawFromQuaternion(terminal_point.pose.orientation) - robot_pose.yaw);
  const double reverse_offset = direction < 0 ? kPi : 0.0;
  target.heading = normalizeAngle(endpoint_heading_error_base + reverse_offset);

  target.valid =
    std::isfinite(target.x) &&
    std::isfinite(target.y) &&
    std::isfinite(target.heading) &&
    std::isfinite(target.distance);
  return target;
}

void LqrController::extendMotionPathForReference(
  nav_msgs::msg::Path & motion_path,
  double minimum_length) const
{
  if (motion_path.poses.size() < 2U) {
    return;
  }

  const double current_length = pathLength(motion_path);
  if (current_length >= minimum_length) {
    return;
  }

  const auto & previous =
    motion_path.poses[motion_path.poses.size() - 2U].pose.position;
  const auto & terminal = motion_path.poses.back().pose.position;
  double dx = terminal.x - previous.x;
  double dy = terminal.y - previous.y;
  double norm = std::hypot(dx, dy);
  if (norm < 1.0e-6) {
    dx = 1.0;
    dy = 0.0;
    norm = 1.0;
  }

  dx /= norm;
  dy /= norm;
  const double extension = minimum_length - current_length + 0.05;

  geometry_msgs::msg::PoseStamped extra = motion_path.poses.back();
  extra.pose.position.x += dx * extension;
  extra.pose.position.y += dy * extension;
  motion_path.poses.push_back(extra);
}

double LqrController::computeAdaptiveLookahead(double target_speed) const
{
  return clamp(
    lookahead_min_distance_ +
    lookahead_speed_gain_ * std::fabs(target_speed),
    lookahead_min_distance_,
    lookahead_max_distance_);
}

bool LqrController::findLookaheadPoint(
  const nav_msgs::msg::Path & local_path,
  double lookahead_distance,
  geometry_msgs::msg::PoseStamped & lookahead_point)
{
  if (local_path.poses.empty()) {
    return false;
  }

  double accumulated_length = 0.0;
  geometry_msgs::msg::PoseStamped previous_pose = local_path.poses.front();
  if (std::hypot(
      previous_pose.pose.position.x,
      previous_pose.pose.position.y) >= lookahead_distance)
  {
    lookahead_point = previous_pose;
    return true;
  }

  for (std::size_t index = 1U; index < local_path.poses.size(); ++index) {
    const auto & current_pose = local_path.poses[index];
    const double segment_length = std::hypot(
      current_pose.pose.position.x - previous_pose.pose.position.x,
      current_pose.pose.position.y - previous_pose.pose.position.y);

    if (accumulated_length + segment_length >= lookahead_distance) {
      const double remaining_length = lookahead_distance - accumulated_length;
      const double ratio = segment_length > kEpsilon ?
        remaining_length / segment_length : 0.0;

      lookahead_point = previous_pose;
      lookahead_point.pose.position.x =
        previous_pose.pose.position.x + ratio *
        (current_pose.pose.position.x - previous_pose.pose.position.x);
      lookahead_point.pose.position.y =
        previous_pose.pose.position.y + ratio *
        (current_pose.pose.position.y - previous_pose.pose.position.y);
      lookahead_point.pose.position.z =
        previous_pose.pose.position.z + ratio *
        (current_pose.pose.position.z - previous_pose.pose.position.z);
      lookahead_point.pose.orientation = current_pose.pose.orientation;
      return true;
    }

    accumulated_length += segment_length;
    previous_pose = current_pose;
  }

  lookahead_point = local_path.poses.back();
  return true;
}

LqrController::ReferenceInfo LqrController::computeReferenceInfo(
  const nav_msgs::msg::Path & local_path,
  int driving_direction_sign,
  double target_speed,
  const TerminalTarget & external_terminal,
  bool terminal_control_enabled)
{
  ReferenceInfo reference;
  reference.driving_direction_sign = driving_direction_sign >= 0 ? 1 : -1;
  reference.curve_anticipation_speed_limit = max_linear_velocity_;
  if (local_path.poses.empty()) {
    return reference;
  }

  nav_msgs::msg::Path motion_path;
  if (!transformPathToMotionFrame(
      local_path,
      driving_direction_sign,
      motion_path))
  {
    return reference;
  }

  const auto & path_terminal = motion_path.poses.back().pose.position;
  const bool use_external_terminal =
    terminal_control_enabled && external_terminal.valid;
  const bool use_path_terminal =
    terminal_control_enabled &&
    terminal_stop_on_path_end_enable_ &&
    !use_external_terminal;

  reference.terminal_is_real_target =
    use_external_terminal || use_path_terminal;
  if (use_external_terminal) {
    reference.terminal_x = external_terminal.x;
    reference.terminal_y = external_terminal.y;
    reference.terminal_heading = external_terminal.heading;
    reference.terminal_distance = external_terminal.distance;
  } else {
    reference.terminal_x = path_terminal.x;
    reference.terminal_y = path_terminal.y;
    reference.terminal_heading = terminalHeadingFromMotionPath(motion_path);
    reference.terminal_distance = std::hypot(path_terminal.x, path_terminal.y);
  }

  if (!std::isfinite(reference.terminal_heading)) {
    reference.terminal_heading = terminalHeadingFromMotionPath(motion_path);
  }

  const double terminal_cosine = std::cos(reference.terminal_heading);
  const double terminal_sine = std::sin(reference.terminal_heading);
  reference.terminal_along_error =
    terminal_cosine * reference.terminal_x +
    terminal_sine * reference.terminal_y;
  reference.terminal_cross_track_error =
    -terminal_sine * reference.terminal_x +
    terminal_cosine * reference.terminal_y;

  nav_msgs::msg::Path filtered_path;
  if (!filterMotionPath(motion_path, filtered_path)) {
    filtered_path = motion_path;
  }

  reference.lookahead_distance = computeAdaptiveLookahead(target_speed);
  extendMotionPathForReference(
    filtered_path,
    std::max(reference.lookahead_distance, minimum_reference_length_));

  if (!findLookaheadPoint(
      filtered_path,
      reference.lookahead_distance,
      reference.lookahead_point))
  {
    return reference;
  }

  const SegmentProjection nearest =
    computeNearestSegmentErrorInMotionFrame(filtered_path);
  if (!nearest.valid) {
    return reference;
  }

  reference.lateral_error =
    std::fabs(nearest.lateral_error) < lateral_error_deadband_ ?
    0.0 : nearest.lateral_error;
  reference.heading_error =
    std::fabs(nearest.heading_error) < heading_error_deadband_ ?
    0.0 : nearest.heading_error;

  reference.current_path_curvature = pathCurvatureAtProgress(
    filtered_path,
    nearest.progress,
    path_curvature_window_distance_);
  reference.preview_path_curvature = previewPathCurvatureAtProgress(
    filtered_path,
    nearest.progress,
    target_speed);
  reference.feedforward_curvature = clamp(
    path_curvature_feedforward_gain_ * reference.preview_path_curvature,
    -max_curvature_,
    max_curvature_);
  reference.curve_anticipation_speed_limit =
    computeCurveAnticipationSpeedLimit(
    filtered_path,
    nearest.progress,
    target_speed);

  const double limited_lateral_error = clamp(
    reference.lateral_error,
    -lateral_error_limit_,
    lateral_error_limit_);
  const double limited_heading_error = clamp(
    reference.heading_error,
    -heading_error_limit_,
    heading_error_limit_);
  reference.target_curvature_estimate = clamp(
    reference.feedforward_curvature +
    speed_curvature_lateral_error_gain_ * limited_lateral_error +
    speed_curvature_heading_error_gain_ * limited_heading_error,
    -max_curvature_,
    max_curvature_);

  reference.valid = true;
  return reference;
}

double LqrController::applyCurvatureSpeedLimit(
  double speed,
  double curvature) const
{
  if (!curvature_speed_limit_enable_ || std::fabs(curvature) < 1.0e-6) {
    return speed;
  }

  const double curvature_limited_speed = std::sqrt(
    max_lateral_acceleration_ / std::max(std::fabs(curvature), 1.0e-6));
  return std::min(speed, curvature_limited_speed);
}

double LqrController::applyPathAcquisitionSpeedLimit(
  double speed,
  double lateral_error,
  double heading_error)
{
  if (!path_acquisition_speed_limit_enable_) {
    path_acquisition_active_ = false;
    path_acquired_once_ = true;
    path_acquisition_stable_count_ = 0;
    return speed;
  }

  const double lateral_abs = std::fabs(lateral_error);
  const double heading_abs = std::fabs(heading_error);

  const bool enter_required = !path_acquired_once_ ?
    (lateral_abs >= path_acquisition_enter_lateral_error_ ||
    heading_abs >= path_acquisition_enter_heading_error_) :
    (lateral_abs >= path_reacquisition_enter_lateral_error_ ||
    heading_abs >= path_reacquisition_enter_heading_error_);

  if (!path_acquisition_active_ && enter_required) {
    path_acquisition_active_ = true;
    path_acquisition_stable_count_ = 0;
  }

  if (path_acquisition_active_) {
    const bool stable =
      lateral_abs <= path_acquisition_exit_lateral_error_ &&
      heading_abs <= path_acquisition_exit_heading_error_;
    if (stable) {
      ++path_acquisition_stable_count_;
      if (path_acquisition_stable_count_ >= path_acquisition_stable_cycles_) {
        path_acquisition_active_ = false;
        path_acquired_once_ = true;
        path_acquisition_stable_count_ = 0;
      }
    } else {
      path_acquisition_stable_count_ = 0;
    }
  }

  if (!path_acquisition_active_) {
    return speed;
  }

  const double lateral_ratio = clamp(
    (lateral_abs - path_acquisition_exit_lateral_error_) /
    std::max(
      path_acquisition_full_lateral_error_ -
      path_acquisition_exit_lateral_error_,
      1.0e-6),
    0.0,
    1.0);
  const double heading_ratio = clamp(
    (heading_abs - path_acquisition_exit_heading_error_) /
    std::max(
      path_acquisition_full_heading_error_ -
      path_acquisition_exit_heading_error_,
      1.0e-6),
    0.0,
    1.0);
  const double severity = std::max(lateral_ratio, heading_ratio);
  const double acquisition_limit =
    path_acquisition_max_speed_ - severity *
    (path_acquisition_max_speed_ - path_acquisition_min_speed_);

  return std::min(speed, acquisition_limit);
}

double LqrController::computeTargetSpeed(const ReferenceInfo & reference) const
{
  double speed = std::min(target_linear_velocity_, max_linear_velocity_);
  speed = applyCurvatureSpeedLimit(
    speed,
    reference.target_curvature_estimate);

  if (error_speed_limit_enable_) {
    const double error_scale = 1.0 /
      (1.0 +
      lateral_error_speed_gain_ * std::fabs(reference.lateral_error) +
      heading_error_speed_gain_ * std::fabs(reference.heading_error));
    speed *= clamp(error_scale, 0.25, 1.0);
  }

  if (terminal_slowdown_enable_ &&
    reference.terminal_is_real_target &&
    reference.terminal_distance < terminal_slowdown_distance_)
  {
    const double signed_s = reference.terminal_along_error;
    const bool position_window_satisfied =
      reference.terminal_distance <= terminal_stop_distance_ &&
      std::fabs(signed_s) <= terminal_stop_s_tolerance_ &&
      std::fabs(reference.terminal_cross_track_error) <=
      terminal_stop_lateral_tolerance_;

    if (position_window_satisfied) {
      return 0.0;
    }

    // base_link has reached or crossed the terminal tangent plane.  Continuing
    // with the current gear can only increase the longitudinal error, so never
    // keep a non-zero command merely to chase a remaining lateral error.
    if (signed_s <= 0.0) {
      return 0.0;
    }

    const double longitudinal_speed_limit =
      terminal_longitudinal_speed_gain_ * signed_s;
    speed = std::min(speed, longitudinal_speed_limit);

    const double lateral_excess = std::max(
      0.0,
      std::fabs(reference.terminal_cross_track_error) -
      terminal_stop_lateral_tolerance_);
    const double distance_excess = std::max(
      0.0,
      reference.terminal_distance - terminal_stop_distance_);

    double correction_speed_floor = terminal_min_linear_velocity_;
    correction_speed_floor +=
      terminal_lateral_error_speed_floor_gain_ * lateral_excess;
    correction_speed_floor +=
      terminal_distance_error_speed_floor_gain_ * distance_excess;
    correction_speed_floor = clamp(
      correction_speed_floor,
      terminal_min_linear_velocity_,
      terminal_correction_speed_max_);

    // The correction floor must never override the longitudinal braking law.
    // The previous implementation forced 0.02-0.04 m/s in the final centimetres
    // whenever cte was non-zero, which is exactly why forward motion crossed the
    // base_link target plane before the 5 mm window was reached.
    const double safe_correction_floor = std::min(
      correction_speed_floor,
      longitudinal_speed_limit);
    if (signed_s > terminal_stop_s_tolerance_) {
      speed = std::max(speed, safe_correction_floor);
    }

  } else if (speed > 1.0e-6 && speed < min_linear_velocity_) {
    speed = min_linear_velocity_;
  }

  return clamp(speed, 0.0, max_linear_velocity_);
}

std::array<double, 4> LqrController::solveDiscreteLqrGain(
  double speed_abs,
  double reference_curvature) const
{
  using Matrix4 = std::array<std::array<double, 4>, 4>;
  using Vector4 = std::array<double, 4>;

  // v and dt belong to the discrete state matrix A, not to Q.
  // A and B are rebuilt every cycle because steering authority changes with speed
  // and with the feedforward steering working point.
  const double model_speed = std::max(
    speed_abs,
    std::max(terminal_min_linear_velocity_, 0.05));
  const double dt = std::max(control_period_, 1.0e-3);
  const double reference_steering =
    curvatureToCentralSteering(reference_curvature);
  const double steering_response =
    2.0 * model_speed * std::cos(reference_steering) /
    std::max(wheel_base_, 1.0e-6);

  // X(k + 1) = A X(k) + B u(k)
  Matrix4 A{};
  A[0][0] = 1.0;
  A[0][1] = dt;
  A[1][2] = model_speed;
  A[2][2] = 1.0;
  A[2][3] = dt;

  // The controller has one input, so B is a 4 x 1 column vector.
  Vector4 B{};
  B[3] = -steering_response;

  Matrix4 Q{};
  Q[0][0] = lqr_q_lateral_;
  Q[1][1] = lqr_q_lateral_rate_;
  Q[2][2] = lqr_q_heading_;
  Q[3][3] = lqr_q_heading_rate_;

  const double R = lqr_r_steering_;

  // Fixed-point iteration of the discrete algebraic Riccati equation:
  // P_next = Q + A^T P A
  //          - A^T P B (R + B^T P B)^-1 B^T P A.
  Matrix4 P = Q;
  for (int iteration = 0; iteration < 160; ++iteration) {
    // PB = P * B
    Vector4 PB{};
    for (int row = 0; row < 4; ++row) {
      for (int inner = 0; inner < 4; ++inner) {
        PB[row] += P[row][inner] * B[inner];
      }
    }

    // S = R + B^T * P * B. S is scalar because the system has one input.
    double BtPB = 0.0;
    for (int inner = 0; inner < 4; ++inner) {
      BtPB += B[inner] * PB[inner];
    }
    const double S = std::max(R + BtPB, 1.0e-9);
    const double inv_S = 1.0 / S;

    // PA = P * A
    Matrix4 PA{};
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        for (int inner = 0; inner < 4; ++inner) {
          PA[row][column] += P[row][inner] * A[inner][column];
        }
      }
    }

    // AtPB = A^T * P * B
    Vector4 AtPB{};
    for (int row = 0; row < 4; ++row) {
      for (int inner = 0; inner < 4; ++inner) {
        AtPB[row] += A[inner][row] * PB[inner];
      }
    }

    // BtPA = B^T * P * A
    Vector4 BtPA{};
    for (int column = 0; column < 4; ++column) {
      for (int inner = 0; inner < 4; ++inner) {
        BtPA[column] += B[inner] * PA[inner][column];
      }
    }

    // AtPA = A^T * P * A
    Matrix4 AtPA{};
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        for (int inner = 0; inner < 4; ++inner) {
          AtPA[row][column] += A[inner][row] * PA[inner][column];
        }
      }
    }

    Matrix4 P_next = Q;
    double max_change = 0.0;
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        P_next[row][column] +=
          AtPA[row][column] -
          AtPB[row] * inv_S * BtPA[column];
        max_change = std::max(
          max_change,
          std::fabs(P_next[row][column] - P[row][column]));
      }
    }

    P = P_next;
    if (max_change < 1.0e-10) {
      break;
    }
  }

  // K = (R + B^T P B)^-1 B^T P A
  Vector4 PB{};
  for (int row = 0; row < 4; ++row) {
    for (int inner = 0; inner < 4; ++inner) {
      PB[row] += P[row][inner] * B[inner];
    }
  }

  double BtPB = 0.0;
  for (int inner = 0; inner < 4; ++inner) {
    BtPB += B[inner] * PB[inner];
  }
  const double S = std::max(R + BtPB, 1.0e-9);
  const double inv_S = 1.0 / S;

  Matrix4 PA{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      for (int inner = 0; inner < 4; ++inner) {
        PA[row][column] += P[row][inner] * A[inner][column];
      }
    }
  }

  Vector4 BtPA{};
  for (int column = 0; column < 4; ++column) {
    for (int inner = 0; inner < 4; ++inner) {
      BtPA[column] += B[inner] * PA[inner][column];
    }
  }

  Vector4 K{};
  for (int column = 0; column < 4; ++column) {
    K[column] = inv_S * BtPA[column];
  }
  return K;
}

double LqrController::computeLqrCurvature(
  const ReferenceInfo & reference,
  double speed_abs)
{
  const double dt = std::max(control_period_, 1.0e-3);
  const double lateral_error = clamp(
    reference.lateral_error,
    -lateral_error_limit_,
    lateral_error_limit_);
  const double heading_error = clamp(
    reference.heading_error,
    -heading_error_limit_,
    heading_error_limit_);

  const double reference_curvature_magnitude = std::max(
    std::fabs(reference.current_path_curvature),
    std::fabs(reference.feedforward_curvature));
  const bool straight_reference =
    reference_curvature_magnitude <=
    straight_reference_curvature_threshold_;

  const double model_lateral_rate =
    speed_abs * std::sin(heading_error);
  const double previous_curvature = has_last_curvature_ ?
    last_curvature_ : reference.current_path_curvature;
  const double model_heading_rate = speed_abs *
    (reference.feedforward_curvature - previous_curvature);

  double lateral_rate_sample = model_lateral_rate;
  double heading_rate_sample = model_heading_rate;
  if (has_error_history_) {
    const double measured_lateral_rate = clamp(
      (reference.lateral_error - last_lateral_error_) / dt,
      -1.0,
      1.0);
    const double measured_heading_rate = clamp(
      normalizeAngle(reference.heading_error - last_heading_error_) / dt,
      -2.0,
      2.0);
    const double measured_rate_blend = straight_reference ? 0.12 : 0.30;
    lateral_rate_sample =
      measured_rate_blend * measured_lateral_rate +
      (1.0 - measured_rate_blend) * model_lateral_rate;
    heading_rate_sample =
      measured_rate_blend * measured_heading_rate +
      (1.0 - measured_rate_blend) * model_heading_rate;
  }

  if (!has_error_history_) {
    filtered_lateral_error_rate_ = lateral_rate_sample;
    filtered_heading_error_rate_ = heading_rate_sample;
  } else {
    filtered_lateral_error_rate_ =
      lqr_error_rate_filter_alpha_ * lateral_rate_sample +
      (1.0 - lqr_error_rate_filter_alpha_) *
      filtered_lateral_error_rate_;
    filtered_heading_error_rate_ =
      lqr_error_rate_filter_alpha_ * heading_rate_sample +
      (1.0 - lqr_error_rate_filter_alpha_) *
      filtered_heading_error_rate_;
  }

  const double lateral_error_rate = clamp(
    filtered_lateral_error_rate_,
    -1.0,
    1.0);
  const double heading_error_rate = clamp(
    filtered_heading_error_rate_,
    -2.0,
    2.0);

  const auto K = solveDiscreteLqrGain(
    speed_abs,
    reference.feedforward_curvature);

  double feedback_scale = lqr_feedback_scale_;
  if (!path_acquisition_active_) {
    feedback_scale *= straight_reference ?
      straight_feedback_scale_ : curve_feedback_scale_;
  }

  double feedback_central_steering = -feedback_scale *
    (K[0] * lateral_error +
    K[1] * lateral_error_rate +
    K[2] * heading_error +
    K[3] * heading_error_rate);
  if (straight_reference && !path_acquisition_active_) {
    feedback_central_steering = clamp(
      feedback_central_steering,
      -straight_feedback_max_central_steering_,
      straight_feedback_max_central_steering_);
  }

  const double feedforward_central_steering =
    curvatureToCentralSteering(reference.feedforward_curvature);
  const double lqr_central_steering = clamp(
    feedforward_central_steering + feedback_central_steering,
    -max_central_steering_angle_,
    max_central_steering_angle_);

  // Keep the terminal steering behaviour of the user-provided smooth version:
  // forward motion remains entirely on the path feedforward + DARE-LQR feedback,
  // while reverse gets only a small proportional lateral trim.  The removed
  // forward terminal-line angle term could reach 0.10 rad and overrode the path
  // tangent near the endpoint, degrading final yaw and producing a late hook.
  double reverse_terminal_trim = 0.0;
  if (reverse_terminal_lateral_convergence_enable_ &&
    reference.driving_direction_sign < 0 &&
    reference.terminal_is_real_target &&
    reference.terminal_distance <
    reverse_terminal_lateral_convergence_start_distance_ &&
    reference.terminal_along_error >= -terminal_stop_s_tolerance_)
  {
    const double blend_span = std::max(
      reverse_terminal_lateral_convergence_start_distance_ -
      reverse_terminal_lateral_convergence_fade_distance_,
      1.0e-6);
    const double entry_blend = clamp(
      (reverse_terminal_lateral_convergence_start_distance_ -
      reference.terminal_distance) /
      blend_span,
      0.0,
      1.0);
    const bool lateral_window_satisfied =
      std::fabs(reference.terminal_cross_track_error) <=
      terminal_stop_lateral_tolerance_;
    const double final_fade = lateral_window_satisfied ?
      clamp(
      reference.terminal_distance /
      std::max(
        reverse_terminal_lateral_convergence_fade_distance_,
        1.0e-6),
      0.0,
      1.0) : 1.0;
    const double lateral_excess = std::max(
      0.0,
      std::fabs(reference.terminal_cross_track_error) -
      reverse_terminal_lateral_convergence_deadband_);

    if (lateral_excess > 0.0) {
      reverse_terminal_trim = std::copysign(
        reverse_terminal_lateral_convergence_gain_ *
        lateral_excess * entry_blend * final_fade,
        reference.terminal_cross_track_error);
      reverse_terminal_trim = clamp(
        reverse_terminal_trim,
        -reverse_terminal_lateral_convergence_max_steering_,
        reverse_terminal_lateral_convergence_max_steering_);
    }
  }

  const double final_central_steering = clamp(
    lqr_central_steering + reverse_terminal_trim,
    -max_central_steering_angle_,
    max_central_steering_angle_);
  const double raw_curvature = clamp(
    centralSteeringToCurvature(final_central_steering),
    -max_curvature_,
    max_curvature_);

  last_lateral_error_ = reference.lateral_error;
  last_heading_error_ = reference.heading_error;
  has_error_history_ = true;

  control_debug_.lqr_gain = K;
  control_debug_.feedforward_central_steering =
    feedforward_central_steering;
  control_debug_.feedback_central_steering =
    feedback_central_steering;
  control_debug_.reverse_terminal_trim = reverse_terminal_trim;
  control_debug_.raw_curvature = raw_curvature;

  return raw_curvature;
}

double LqrController::smoothCurvature(
  double raw_curvature,
  int driving_direction_sign,
  double reference_curvature)
{
  if (!curvature_smoothing_enable_) {
    last_curvature_ = raw_curvature;
    has_last_curvature_ = true;
    last_direction_ = driving_direction_sign;
    return raw_curvature;
  }

  if (!has_last_curvature_ || last_direction_ != driving_direction_sign) {
    last_curvature_ = raw_curvature;
    has_last_curvature_ = true;
    last_direction_ = driving_direction_sign;
    return raw_curvature;
  }

  const bool sign_change =
    raw_curvature * last_curvature_ < 0.0 &&
    std::fabs(raw_curvature) > curve_preview_activation_curvature_ &&
    std::fabs(last_curvature_) > curve_preview_activation_curvature_;
  const bool increasing_turn =
    !sign_change &&
    std::fabs(raw_curvature) > std::fabs(last_curvature_) + 1.0e-4;
  const bool leaving_turn =
    !sign_change &&
    std::fabs(raw_curvature) + 1.0e-4 < std::fabs(last_curvature_);
  const bool straight_reference =
    std::fabs(reference_curvature) <=
    straight_reference_curvature_threshold_;

  double alpha = curvature_smoothing_alpha_;
  double rate_multiplier = 1.0;
  if (straight_reference) {
    if (sign_change) {
      alpha = straight_sign_change_smoothing_alpha_;
      rate_multiplier = straight_sign_change_rate_multiplier_;
    }
  } else if (sign_change || increasing_turn) {
    alpha = std::max(alpha, curve_entry_smoothing_alpha_);
    rate_multiplier =
      curve_entry_rate_multiplier_ * (sign_change ? 1.10 : 1.0);
  } else if (leaving_turn) {
    alpha = std::max(alpha, curve_exit_smoothing_alpha_);
  }

  const double max_delta =
    max_curvature_rate_ * rate_multiplier * control_period_;
  const double rate_limited = last_curvature_ + clamp(
    raw_curvature - last_curvature_,
    -max_delta,
    max_delta);
  const double smoothed =
    alpha * rate_limited + (1.0 - alpha) * last_curvature_;

  last_curvature_ = clamp(smoothed, -max_curvature_, max_curvature_);
  return last_curvature_;
}

geometry_msgs::msg::Twist LqrController::smoothCommand(
  const geometry_msgs::msg::Twist & raw_cmd)
{
  if (!command_smoothing_enable_ || !has_last_command_) {
    last_cmd_ = raw_cmd;
    has_last_command_ = true;
    return raw_cmd;
  }

  geometry_msgs::msg::Twist smoothed = raw_cmd;
  double linear_alpha = command_smoothing_alpha_;
  if (std::fabs(raw_cmd.linear.x) < std::fabs(last_cmd_.linear.x)) {
    linear_alpha = std::max(
      command_smoothing_alpha_,
      command_deceleration_alpha_);
  }

  double linear_target = raw_cmd.linear.x;
  const bool accelerating_same_direction =
    raw_cmd.linear.x * last_cmd_.linear.x >= 0.0 &&
    std::fabs(raw_cmd.linear.x) >
    std::fabs(last_cmd_.linear.x) + 1.0e-9;
  if (accelerating_same_direction && max_linear_acceleration_ > 1.0e-9) {
    const double max_linear_delta =
      max_linear_acceleration_ * control_period_;
    linear_target = last_cmd_.linear.x + clamp(
      raw_cmd.linear.x - last_cmd_.linear.x,
      -max_linear_delta,
      max_linear_delta);
  }

  smoothed.linear.x =
    linear_alpha * linear_target +
    (1.0 - linear_alpha) * last_cmd_.linear.x;

  // Ranger derives steering from angular.z / |linear.x|.  Preserve this ratio
  // after speed smoothing instead of filtering angular.z independently.
  const double raw_speed_abs = std::fabs(raw_cmd.linear.x);
  if (raw_speed_abs > 1.0e-6 && std::fabs(smoothed.linear.x) > 1.0e-9) {
    const double yaw_rate_ratio = raw_cmd.angular.z / raw_speed_abs;
    smoothed.angular.z = clamp(
      std::fabs(smoothed.linear.x) * yaw_rate_ratio,
      -max_angular_velocity_,
      max_angular_velocity_);
  } else {
    smoothed.angular.z = 0.0;
  }

  last_cmd_ = smoothed;
  return smoothed;
}

void LqrController::publishDiagnostics(
  const ReferenceInfo & reference,
  const geometry_msgs::msg::Twist & cmd_vel,
  double smoothed_curvature,
  int direction) const
{
  if (!tracking_error_pub_) {
    return;
  }

  std_msgs::msg::Float64MultiArray message;
  message.data = {
    reference.lateral_error,                  // 0: e_y [m]
    reference.heading_error,                  // 1: e_psi [rad]
    reference.terminal_distance,              // 2: terminal distance [m]
    cmd_vel.linear.x,                         // 3: command speed [m/s]
    cmd_vel.angular.z,                        // 4: command yaw rate [rad/s]
    control_debug_.lqr_gain[0],               // 5: K_e_y
    control_debug_.lqr_gain[1],               // 6: K_e_y_dot
    control_debug_.lqr_gain[2],               // 7: K_e_psi
    control_debug_.lqr_gain[3],               // 8: K_e_psi_dot
    reference.lookahead_distance,             // 9: lookahead distance [m]
    static_cast<double>(direction),            // 10: direction sign
    reference.terminal_x,                     // 11: terminal x in motion frame [m]
    reference.terminal_y,                     // 12: terminal y in motion frame [m]
    reference.terminal_heading,               // 13: terminal tangent [rad]
    control_debug_.reverse_terminal_trim,      // 14: reverse trim [rad]
    reference.terminal_is_real_target ? 1.0 : 0.0,  // 15
    reference.terminal_along_error,            // 16: terminal s [m]
    reference.terminal_cross_track_error,      // 17: terminal cte [m]
    terminal_stop_latched_ ? 1.0 : 0.0,        // 18
    reference.current_path_curvature,          // 19: kappa now [1/m]
    reference.preview_path_curvature,          // 20: kappa short preview [1/m]
    reference.feedforward_curvature,           // 21: kappa feedforward [1/m]
    control_debug_.raw_curvature,              // 22: raw commanded curvature [1/m]
    smoothed_curvature                         // 23: smoothed curvature [1/m]
  };
  tracking_error_pub_->publish(message);
}

bool LqrController::computeVelocityCommand(
  const nav_msgs::msg::Odometry & odom,
  const nav_msgs::msg::Path & local_path,
  const geometry_msgs::msg::PoseStamped & tracking_point,
  int driving_direction_sign,
  geometry_msgs::msg::Twist & cmd_vel)
{
  return computeVelocityCommand(
    odom,
    local_path,
    tracking_point,
    driving_direction_sign,
    true,
    cmd_vel);
}

bool LqrController::computeVelocityCommand(
  const nav_msgs::msg::Odometry & odom,
  const nav_msgs::msg::Path & local_path,
  const geometry_msgs::msg::PoseStamped & tracking_point,
  int driving_direction_sign,
  bool terminal_target_enabled,
  geometry_msgs::msg::Twist & cmd_vel)
{
  cmd_vel = geometry_msgs::msg::Twist{};
  if (!initialized_) {
    reset();
    return false;
  }

  nav_msgs::msg::Path effective_path = local_path;
  if (effective_path.poses.size() < 2U) {
    if (!buildFallbackPath(odom, tracking_point, effective_path)) {
      reset();
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[%s] Local path has fewer than two poses; using a tangent fallback path.",
      name_.c_str());
  }

  const int direction = driving_direction_sign >= 0 ? 1 : -1;
  const bool terminal_control_enabled =
    terminal_target_enabled && !navCoreParkingAdjustmentEnabled(node_);
  const TerminalTarget external_terminal = terminal_control_enabled ?
    computeExternalTerminalTarget(odom, tracking_point, direction) :
    TerminalTarget{};

  if (!terminal_control_enabled) {
    terminal_hold_count_ = 0;
    terminal_stop_latched_ = false;
    terminal_stop_result_printed_ = false;
  }

  // Once base_link first enters the 5 mm window, hold zero with a 3 cm
  // hysteresis band while confirming the stop.  This prevents centimetre-level
  // localization noise from alternating the command between zero and non-zero.
  // The candidate and the final latch both use the same base_link terminal
  // target that is used by the path tracker.
  if (terminal_control_enabled &&
    terminal_stop_latch_enable_ &&
    terminal_hold_count_ > 0 &&
    external_terminal.valid)
  {
    if (external_terminal.distance <= terminal_stop_exit_distance_) {
      terminal_hold_count_ = std::min(
        terminal_hold_count_ + 1,
        terminal_stop_hold_cycles_);
      terminal_stop_latched_ =
        terminal_hold_count_ >= terminal_stop_hold_cycles_;
      last_cmd_ = geometry_msgs::msg::Twist{};
      has_last_command_ = true;
      cmd_vel = last_cmd_;

      // Continue publishing the exact final base_link error while the zero-speed
      // hold is being confirmed. Previously this branch returned before updating
      // ~/tracking_error, which made the last displayed error look stale.
      ReferenceInfo hold_reference;
      hold_reference.valid = true;
      hold_reference.driving_direction_sign = direction;
      hold_reference.terminal_is_real_target = true;
      hold_reference.terminal_x = external_terminal.x;
      hold_reference.terminal_y = external_terminal.y;
      hold_reference.terminal_heading = external_terminal.heading;
      hold_reference.terminal_distance = external_terminal.distance;
      const double terminal_cosine = std::cos(external_terminal.heading);
      const double terminal_sine = std::sin(external_terminal.heading);
      hold_reference.terminal_along_error =
        terminal_cosine * external_terminal.x +
        terminal_sine * external_terminal.y;
      hold_reference.terminal_cross_track_error =
        -terminal_sine * external_terminal.x +
        terminal_cosine * external_terminal.y;
      publishDiagnostics(hold_reference, cmd_vel, 0.0, direction);
      return true;
    }
    terminal_stop_latched_ = false;
    terminal_hold_count_ = 0;
  }

  // First pass: use maximum target speed to establish preview distances.
  ReferenceInfo reference = computeReferenceInfo(
    effective_path,
    direction,
    std::min(target_linear_velocity_, max_linear_velocity_),
    external_terminal,
    terminal_control_enabled);
  if (!reference.valid) {
    reset();
    return false;
  }

  double first_pass_speed = computeTargetSpeed(reference);
  first_pass_speed = std::min(
    first_pass_speed,
    reference.curve_anticipation_speed_limit);
  first_pass_speed = applyPathAcquisitionSpeedLimit(
    first_pass_speed,
    reference.lateral_error,
    reference.heading_error);

  // Second pass: recompute speed-dependent short and long previews using the
  // speed that will actually be allowed by terminal, error and curve limits.
  reference = computeReferenceInfo(
    effective_path,
    direction,
    first_pass_speed,
    external_terminal,
    terminal_control_enabled);
  if (!reference.valid) {
    reset();
    return false;
  }

  geometry_msgs::msg::PoseStamped published_lookahead =
    reference.lookahead_point;
  published_lookahead.pose.position.x =
    static_cast<double>(direction) * reference.lookahead_point.pose.position.x;
  published_lookahead.pose.position.y =
    static_cast<double>(direction) * reference.lookahead_point.pose.position.y;
  published_lookahead.header.frame_id = robot_base_.base_frame;
  published_lookahead.header.stamp = node_->now();
  if (lookahead_point_pub_) {
    lookahead_point_pub_->publish(published_lookahead);
  }

  double speed_abs = computeTargetSpeed(reference);
  speed_abs = std::min(
    speed_abs,
    reference.curve_anticipation_speed_limit);
  if (path_acquisition_active_) {
    speed_abs = std::min(speed_abs, first_pass_speed);
  }

  if (std::fabs(reference.target_curvature_estimate) > 1.0e-3) {
    const double steering_demand_speed_limit = std::sqrt(
      0.35 /
      std::max(std::fabs(reference.target_curvature_estimate), 1.0e-3));
    speed_abs = std::min(speed_abs, steering_demand_speed_limit);
  }

  if (!reference.terminal_is_real_target) {
    terminal_hold_count_ = 0;
    terminal_stop_latched_ = false;
  }

  if (terminal_slowdown_enable_ &&
    reference.terminal_is_real_target &&
    reference.terminal_distance < terminal_slowdown_distance_)
  {
    const bool terminal_position_satisfied =
      reference.terminal_distance <= terminal_stop_distance_ &&
      std::fabs(reference.terminal_along_error) <=
      terminal_stop_s_tolerance_ &&
      std::fabs(reference.terminal_cross_track_error) <=
      terminal_stop_lateral_tolerance_;

    if (terminal_position_satisfied) {
      terminal_hold_count_ = terminal_stop_latch_enable_ ? 1 : 0;
      terminal_stop_latched_ =
        terminal_stop_latch_enable_ && terminal_stop_hold_cycles_ <= 1;
      last_cmd_ = geometry_msgs::msg::Twist{};
      has_last_command_ = true;
      cmd_vel = last_cmd_;

      publishDiagnostics(reference, cmd_vel, 0.0, direction);
      // nav_core prints the single authoritative final-stop line after it has
      // verified both this zero command and the same base_link distance window.
      terminal_stop_result_printed_ = true;
      return true;
    }

    terminal_hold_count_ = 0;
    terminal_stop_latched_ = false;
  } else {
    terminal_hold_count_ = 0;
  }

  geometry_msgs::msg::Twist raw_cmd;
  raw_cmd.linear.x = static_cast<double>(direction) * speed_abs;
  if (speed_abs <= 1.0e-9) {
    last_cmd_ = raw_cmd;
    has_last_command_ = true;
    cmd_vel = raw_cmd;
    return true;
  }

  const double raw_curvature = computeLqrCurvature(reference, speed_abs);
  const double smoothed_curvature = smoothCurvature(
    raw_curvature,
    direction,
    reference.feedforward_curvature);

  raw_cmd.linear.x = clamp(
    raw_cmd.linear.x,
    -max_linear_velocity_,
    max_linear_velocity_);
  if (!robot_base_.allow_reverse && raw_cmd.linear.x < 0.0) {
    raw_cmd.linear.x = 0.0;
  }
  raw_cmd.angular.z = twistYawRateFromCurvature(
    speed_abs,
    smoothed_curvature);

  cmd_vel = smoothCommand(raw_cmd);

  const bool terminal_position_satisfied_after_command =
    reference.terminal_is_real_target &&
    reference.terminal_distance <= terminal_stop_distance_ &&
    std::fabs(reference.terminal_along_error) <=
    terminal_stop_s_tolerance_ &&
    std::fabs(reference.terminal_cross_track_error) <=
    terminal_stop_lateral_tolerance_;

  double effective_minimum_speed = terminal_min_linear_velocity_;
  if (reference.terminal_is_real_target &&
    !terminal_position_satisfied_after_command)
  {
    effective_minimum_speed = std::max(
      effective_minimum_speed,
      terminal_correction_min_velocity_);
  }

  if (std::fabs(raw_cmd.linear.x) >= effective_minimum_speed - 1.0e-9 &&
    std::fabs(cmd_vel.linear.x) > 1.0e-9 &&
    std::fabs(cmd_vel.linear.x) < effective_minimum_speed)
  {
    cmd_vel.linear.x = std::copysign(
      effective_minimum_speed,
      raw_cmd.linear.x);
    cmd_vel.angular.z = twistYawRateFromCurvature(
      std::fabs(cmd_vel.linear.x),
      smoothed_curvature);
    last_cmd_ = cmd_vel;
  }

  publishDiagnostics(reference, cmd_vel, smoothed_curvature, direction);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "[%s] DARE-LQR cmd: v=%.3f, wz=%.3f, kappa=%.3f, "
    "e_y=%.3f, e_yaw=%.3f, "
    "reference=base_link_to_base_link, reverse_trim=%.4f, "
    "lookahead=%.3f, k_now=%.3f, "
    "k_preview=%.3f, k_ff=%.3f, stop_latched=%s, direction=%d, acquisition=%s.",
    name_.c_str(),
    cmd_vel.linear.x,
    cmd_vel.angular.z,
    smoothed_curvature,
    reference.lateral_error,
    reference.heading_error,
    control_debug_.reverse_terminal_trim,
    reference.lookahead_distance,
    reference.current_path_curvature,
    reference.preview_path_curvature,
    reference.feedforward_curvature,
    terminal_stop_latched_ ? "true" : "false",
    direction,
    path_acquisition_active_ ? "true" : "false");

  return true;
}

std::string LqrController::getName() const
{
  return name_;
}

}  // namespace lqr

PLUGINLIB_EXPORT_CLASS(lqr::LqrController, nav_core::ControllerPlugin)

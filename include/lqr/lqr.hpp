#ifndef LQR__LQR_HPP_
#define LQR__LQR_HPP_

#include <array>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>

#include "controller_plugin.hpp"

namespace lqr
{

class LqrController : public nav_core::ControllerPlugin
{
public:
  LqrController() = default;
  ~LqrController() override = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node,
    const std::string & plugin_name,
    tf2_ros::Buffer * tf_buffer,
    const nav_core::RobotBaseConfig & robot_base) override;

  void reset() override;

  bool computeVelocityCommand(
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Path & local_path,
    const geometry_msgs::msg::PoseStamped & tracking_point,
    int driving_direction_sign,
    geometry_msgs::msg::Twist & cmd_vel) override;

  bool computeVelocityCommand(
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Path & local_path,
    const geometry_msgs::msg::PoseStamped & tracking_point,
    int driving_direction_sign,
    bool terminal_target_enabled,
    geometry_msgs::msg::Twist & cmd_vel) override;

  std::string getName() const override;

private:
  struct TerminalTarget
  {
    bool valid{false};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double distance{0.0};
  };

  struct ReferenceInfo
  {
    bool valid{false};
    geometry_msgs::msg::PoseStamped lookahead_point;

    double lateral_error{0.0};
    double heading_error{0.0};

    double current_path_curvature{0.0};
    double preview_path_curvature{0.0};
    double feedforward_curvature{0.0};
    double target_curvature_estimate{0.0};
    double curve_anticipation_speed_limit{0.0};

    bool terminal_is_real_target{false};
    double terminal_x{0.0};
    double terminal_y{0.0};
    double terminal_heading{0.0};
    double terminal_distance{0.0};
    double terminal_along_error{0.0};
    double terminal_cross_track_error{0.0};

    double lookahead_distance{0.0};
    int driving_direction_sign{1};
  };

  struct ControlDebug
  {
    std::array<double, 4> lqr_gain{};
    double feedforward_central_steering{0.0};
    double feedback_central_steering{0.0};
    double reverse_terminal_trim{0.0};
    double raw_curvature{0.0};
  };

  template<typename T>
  T declareParameter(const std::string & name, const T & default_value);

  static double clamp(double value, double min_value, double max_value);
  static double normalizeAngle(double angle);

  bool buildFallbackPath(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & tracking_point,
    nav_msgs::msg::Path & effective_path) const;

  bool transformPathToMotionFrame(
    const nav_msgs::msg::Path & local_path,
    int driving_direction_sign,
    nav_msgs::msg::Path & motion_path) const;

  bool filterMotionPath(
    const nav_msgs::msg::Path & motion_path,
    nav_msgs::msg::Path & filtered_path) const;

  static double pathLength(const nav_msgs::msg::Path & path);

  double terminalHeadingFromMotionPath(const nav_msgs::msg::Path & motion_path) const;

  double pathCurvatureAtProgress(
    const nav_msgs::msg::Path & path,
    double progress,
    double window_distance) const;

  double previewPathCurvatureAtProgress(
    const nav_msgs::msg::Path & path,
    double progress,
    double target_speed) const;

  double computeCurveAnticipationSpeedLimit(
    const nav_msgs::msg::Path & path,
    double progress,
    double target_speed) const;

  TerminalTarget computeExternalTerminalTarget(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & terminal_point,
    int driving_direction_sign) const;

  void extendMotionPathForReference(
    nav_msgs::msg::Path & motion_path,
    double minimum_length) const;

  double computeAdaptiveLookahead(double target_speed) const;

  static bool findLookaheadPoint(
    const nav_msgs::msg::Path & local_path,
    double lookahead_distance,
    geometry_msgs::msg::PoseStamped & lookahead_point);

  ReferenceInfo computeReferenceInfo(
    const nav_msgs::msg::Path & local_path,
    int driving_direction_sign,
    double target_speed,
    const TerminalTarget & external_terminal,
    bool terminal_control_enabled);

  double applyCurvatureSpeedLimit(double speed, double curvature) const;

  double applyPathAcquisitionSpeedLimit(
    double speed,
    double lateral_error,
    double heading_error);

  double computeTargetSpeed(const ReferenceInfo & ref) const;

  std::array<double, 4> solveDiscreteLqrGain(
    double speed_abs,
    double reference_curvature) const;

  double innerSteeringToCentral(double inner_steering) const;
  double centralSteeringToInner(double central_steering) const;
  double curvatureToCentralSteering(double curvature) const;
  double centralSteeringToCurvature(double central_steering) const;
  double twistYawRateFromCurvature(double speed_abs, double curvature) const;

  double computeLqrCurvature(const ReferenceInfo & ref, double speed_abs);

  double smoothCurvature(
    double raw_curvature,
    int driving_direction_sign,
    double reference_curvature);

  geometry_msgs::msg::Twist smoothCommand(const geometry_msgs::msg::Twist & raw_cmd);

  void publishDiagnostics(
    const ReferenceInfo & ref,
    const geometry_msgs::msg::Twist & cmd_vel,
    double smoothed_curvature,
    int direction) const;

  rclcpp::Node::SharedPtr node_;
  std::string name_{"lqr"};
  bool initialized_{false};

  nav_core::RobotBaseConfig robot_base_;

  // Vehicle limits and geometry.
  double target_linear_velocity_{1.0};
  double min_linear_velocity_{0.06};
  double max_linear_velocity_{1.0};
  double max_angular_velocity_{1.20};
  double max_linear_acceleration_{0.80};

  double wheel_base_{0.494};
  double track_width_{0.364};
  double max_steering_angle_{0.698};
  double max_central_steering_angle_{0.48};
  double min_turning_radius_{0.59};
  double dual_ackermann_mode_radius_margin_{0.03};
  double max_curvature_{1.0};

  // Four-state DARE-LQR weights and feedback scheduling.
  double lqr_q_lateral_{12.0};
  double lqr_q_lateral_rate_{0.30};
  double lqr_q_heading_{10.0};
  double lqr_q_heading_rate_{0.25};
  double lqr_r_steering_{4.2};
  double lqr_feedback_scale_{1.0};
  double lqr_error_rate_filter_alpha_{0.25};

  double straight_reference_curvature_threshold_{0.025};
  double straight_feedback_scale_{0.55};
  double curve_feedback_scale_{1.10};
  double straight_feedback_max_central_steering_{0.085};

  double lateral_error_limit_{0.25};
  double heading_error_limit_{0.70};
  double lateral_error_deadband_{0.0015};
  double heading_error_deadband_{0.0020};

  // Curvature feedforward and short steering preview.
  double path_curvature_feedforward_gain_{1.0};
  double path_curvature_window_distance_{0.18};
  bool curve_preview_enable_{true};
  double curve_preview_min_distance_{0.08};
  double curve_preview_speed_gain_{0.25};
  double curve_preview_max_distance_{0.38};
  int curve_preview_samples_{7};
  double curve_preview_blend_{0.50};
  double curve_preview_activation_curvature_{0.030};

  // Long preview used only for speed planning.
  bool curve_anticipation_enable_{true};
  double curve_anticipation_min_distance_{0.90};
  double curve_anticipation_speed_gain_{1.50};
  double curve_anticipation_max_distance_{2.50};
  int curve_anticipation_samples_{20};
  double curve_anticipation_curvature_threshold_{0.050};
  double curve_anticipation_lateral_acceleration_{0.12};
  double curve_anticipation_min_speed_{0.24};
  double curve_anticipation_deceleration_{0.65};
  double curve_anticipation_safety_distance_{0.25};

  // Lookahead and local path preparation.
  double lookahead_min_distance_{0.30};
  double lookahead_max_distance_{0.85};
  double lookahead_speed_gain_{0.42};
  double direction_filter_margin_{0.08};
  double minimum_reference_length_{0.40};
  double terminal_heading_lookback_distance_{0.65};

  // Path acquisition speed gate.
  bool path_acquisition_speed_limit_enable_{true};
  double path_acquisition_enter_lateral_error_{0.080};
  double path_acquisition_exit_lateral_error_{0.025};
  double path_acquisition_full_lateral_error_{0.350};
  double path_acquisition_enter_heading_error_{0.220};
  double path_acquisition_exit_heading_error_{0.080};
  double path_acquisition_full_heading_error_{0.800};
  double path_reacquisition_enter_lateral_error_{0.250};
  double path_reacquisition_enter_heading_error_{0.550};
  double path_acquisition_min_speed_{0.120};
  double path_acquisition_max_speed_{0.550};
  int path_acquisition_stable_cycles_{3};
  bool path_acquisition_active_{false};
  bool path_acquired_once_{false};
  int path_acquisition_stable_count_{0};

  // Speed limiting.
  bool curvature_speed_limit_enable_{true};
  double max_lateral_acceleration_{0.32};
  bool error_speed_limit_enable_{true};
  double lateral_error_speed_gain_{0.55};
  double heading_error_speed_gain_{0.38};
  double speed_curvature_lateral_error_gain_{0.36};
  double speed_curvature_heading_error_gain_{0.66};

  // Natural terminal slowdown and position-only stop latch.
  bool terminal_stop_on_path_end_enable_{true};
  bool terminal_stop_latch_enable_{true};
  bool terminal_slowdown_enable_{true};
  double terminal_slowdown_distance_{2.00};
  double terminal_stop_distance_{0.010};
  double terminal_stop_exit_distance_{0.030};
  int terminal_stop_hold_cycles_{3};
  double terminal_min_linear_velocity_{0.010};
  double terminal_longitudinal_speed_gain_{0.50};
  double terminal_stop_s_tolerance_{0.010};
  double terminal_stop_lateral_tolerance_{0.010};
  double terminal_correction_min_velocity_{0.010};
  double terminal_correction_speed_max_{0.040};
  double terminal_lateral_error_speed_floor_gain_{1.80};
  double terminal_distance_error_speed_floor_gain_{0.30};

  // Reverse-only low-speed terminal lateral convergence.
  bool reverse_terminal_lateral_convergence_enable_{true};
  double reverse_terminal_lateral_convergence_start_distance_{1.50};
  double reverse_terminal_lateral_convergence_fade_distance_{0.08};
  double reverse_terminal_lateral_convergence_deadband_{0.004};
  double reverse_terminal_lateral_convergence_gain_{1.35};
  double reverse_terminal_lateral_convergence_max_steering_{0.035};

  // Curvature and command smoothing.
  bool curvature_smoothing_enable_{true};
  double curvature_smoothing_alpha_{0.55};
  double curve_entry_smoothing_alpha_{0.85};
  double curve_exit_smoothing_alpha_{0.72};
  double curve_entry_rate_multiplier_{1.40};
  double max_curvature_rate_{3.00};
  double straight_sign_change_smoothing_alpha_{0.30};
  double straight_sign_change_rate_multiplier_{0.45};
  double control_period_{0.10};

  bool command_smoothing_enable_{true};
  double command_smoothing_alpha_{0.65};
  double command_deceleration_alpha_{1.00};

  // Controller state.
  bool has_last_command_{false};
  geometry_msgs::msg::Twist last_cmd_;

  bool has_last_curvature_{false};
  double last_curvature_{0.0};
  int last_direction_{0};

  bool has_error_history_{false};
  double last_lateral_error_{0.0};
  double last_heading_error_{0.0};
  double filtered_lateral_error_rate_{0.0};
  double filtered_heading_error_rate_{0.0};

  int terminal_hold_count_{0};
  bool terminal_stop_latched_{false};
  bool terminal_stop_result_printed_{false};

  ControlDebug control_debug_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_pub_;
};

}  // namespace lqr

#endif  // LQR__LQR_HPP_

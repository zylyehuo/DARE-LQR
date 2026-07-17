#ifndef LQR__LQR_HPP_
#define LQR__LQR_HPP_

#include <array>
#include <string>
#include <vector>

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
    double nominal_feedforward_curvature{0.0};
    double feedforward_curvature{0.0};
    double feedforward_execution_gain{1.0};
    double feedforward_compensation_scale{1.0};
    double steering_delay_preview_distance{0.0};
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
    double straight_bias_central_steering{0.0};
    double forward_terminal_trim{0.0};
    double reverse_terminal_trim{0.0};
    double empirical_curvature_rate_limit{0.0};
    double post_curve_recovery_remaining{0.0};
    double terminal_alignment_speed_limit{0.0};
    double terminal_monotonic_speed_cap{0.0};
    double reverse_convergence_heading_target{0.0};
    double path_acquisition_heading_target{0.0};
    double reverse_straight_speed_limit{0.0};
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
    double target_speed,
    int driving_direction_sign) const;

  double steeringDelayForCurvature(
    double curvature,
    int driving_direction_sign) const;
  double interpolateEmpiricalYawGain(double absolute_yaw_rate) const;
  double compensateFeedforwardCurvature(
    double nominal_curvature,
    double speed_abs,
    double & execution_gain,
    double & compensation_scale) const;
  double computeStraightSteeringBias(const ReferenceInfo & reference) const;
  double computeTerminalLateralTrim(
    const ReferenceInfo & reference,
    bool forward) const;
  double computeEmpiricalBrakingSpeedLimit(double available_distance) const;
  bool terminalAlignmentActive(const ReferenceInfo & reference) const;
  double terminalAlignmentSpeedLimit(const ReferenceInfo & reference) const;
  void updatePostCurveRecoveryState(
    const ReferenceInfo & reference,
    double measured_speed_abs,
    int driving_direction_sign);

  void updateReverseStraightStabilizationState(
    const ReferenceInfo & reference);

  double reverseStraightSpeedLimit(
    const ReferenceInfo & reference) const;

  double computeReverseConvergenceHeadingTarget(
    const ReferenceInfo & reference,
    bool straight_reference) const;

  double computePathAcquisitionConvergenceHeadingTarget(
    double lateral_error,
    double reference_curvature) const;

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
    double heading_error,
    double reference_curvature);

  double computeTargetSpeed(const ReferenceInfo & ref) const;

  double applyTerminalMonotonicSpeedProfile(
    double speed,
    const ReferenceInfo & ref);

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
    double reference_curvature,
    double speed_abs,
    bool terminal_is_real_target,
    double terminal_distance,
    double lateral_error,
    double heading_error);

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
  double max_linear_deceleration_{0.80};

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
  double curve_feedback_scale_{1.18};
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

  // Chassis-specific empirical steering model.
  bool empirical_feedforward_compensation_enable_{true};
  std::vector<double> empirical_yaw_rate_breakpoints_{
    0.0, 0.02, 0.05, 0.10, 0.15, 0.20};
  std::vector<double> empirical_yaw_rate_gains_{
    1.0, 0.92, 0.92, 0.89, 0.855, 0.82};
  double empirical_feedforward_compensation_strength_{0.70};
  double empirical_feedforward_compensation_max_scale_{1.18};
  double empirical_feedforward_compensation_min_yaw_rate_{0.010};

  bool steering_delay_compensation_enable_{true};
  double positive_curvature_steering_delay_{0.18};
  double negative_curvature_steering_delay_{0.09};
  double steering_delay_compensation_strength_{0.70};
  double steering_delay_max_preview_distance_{0.18};

  // Post-curve straight recovery. The short straight after a bend must not be
  // traversed at full speed before lateral/heading error has settled.
  bool post_curve_recovery_enable_{true};
  double post_curve_enter_curvature_{0.060};
  double post_curve_exit_curvature_{0.025};
  double post_curve_recovery_distance_{2.40};
  double post_curve_recovery_speed_limit_{1.00};
  double post_curve_feedback_scale_{0.85};

  // Reverse straight stabilization. The latest closed-loop bags show a stable
  // 14-18 mm reverse lateral offset on the long final straight. A direction-
  // specific speed/feedback schedule plus a virtual convergence heading removes
  // the steady offset without changing the already-good forward behavior.
  bool reverse_straight_stabilization_enable_{true};
  double reverse_straight_feedback_scale_{0.65};
  double reverse_straight_speed_limit_{1.00};
  double reverse_straight_lateral_enter_{0.010};
  double reverse_straight_lateral_exit_{0.005};
  double reverse_straight_heading_enter_{0.020};
  double reverse_straight_heading_exit_{0.008};
  double reverse_straight_feedback_max_central_steering_{0.040};

  bool reverse_convergence_heading_enable_{true};
  double reverse_convergence_heading_gain_{0.55};
  double reverse_convergence_heading_max_{0.025};
  double reverse_convergence_heading_deadband_{0.004};
  double reverse_convergence_straight_lookahead_{1.00};
  double reverse_convergence_terminal_start_distance_{1.60};
  double reverse_convergence_terminal_lookahead_floor_{0.45};

  bool straight_steering_bias_enable_{false};
  double straight_steering_bias_central_angle_{0.00147};
  double straight_steering_bias_activation_curvature_{0.020};
  double straight_steering_bias_max_lateral_error_{0.050};
  double straight_steering_bias_max_heading_error_{0.080};
  double straight_steering_bias_terminal_fade_distance_{0.60};

  // Long preview used only for speed planning.
  bool curve_anticipation_enable_{true};
  double curve_anticipation_min_distance_{0.90};
  double curve_anticipation_speed_gain_{1.50};
  double curve_anticipation_max_distance_{2.50};
  int curve_anticipation_samples_{20};
  double curve_anticipation_curvature_threshold_{0.050};
  double curve_anticipation_lateral_acceleration_{0.10};
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
  double path_acquisition_enter_lateral_error_{0.050};
  double path_acquisition_exit_lateral_error_{0.012};
  double path_acquisition_full_lateral_error_{0.250};
  double path_acquisition_enter_heading_error_{0.100};
  double path_acquisition_exit_heading_error_{0.025};
  double path_acquisition_full_heading_error_{0.400};
  double path_reacquisition_enter_lateral_error_{0.080};
  double path_reacquisition_enter_heading_error_{0.180};
  double path_acquisition_min_speed_{0.250};
  double path_acquisition_max_speed_{0.700};
  int path_acquisition_stable_cycles_{5};
  bool path_acquisition_convergence_heading_enable_{true};
  double path_acquisition_convergence_heading_gain_{0.80};
  double path_acquisition_convergence_heading_max_{0.140};
  double path_acquisition_convergence_heading_deadband_{0.008};
  double path_acquisition_convergence_lookahead_{1.20};
  double path_acquisition_convergence_max_reference_curvature_{0.050};
  double path_acquisition_lateral_feedback_scale_{0.45};
  double path_acquisition_feedback_scale_{0.75};
  double path_acquisition_max_central_steering_{0.120};
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
  int terminal_stop_hold_cycles_{1};
  double terminal_min_linear_velocity_{0.010};
  double terminal_longitudinal_speed_gain_{0.50};
  double terminal_stop_s_tolerance_{0.010};
  double terminal_stop_lateral_tolerance_{0.010};
  double terminal_correction_min_velocity_{0.010};
  double terminal_correction_speed_max_{0.030};
  double terminal_lateral_error_speed_floor_gain_{1.20};
  double terminal_distance_error_speed_floor_gain_{0.20};

  bool empirical_braking_speed_limit_enable_{true};
  double empirical_braking_quadratic_coefficient_{0.30};
  double empirical_braking_linear_coefficient_{0.085};
  double empirical_braking_safety_margin_{0.0};
  double empirical_braking_activation_distance_{0.25};

  // Terminal line alignment is a smooth speed/feedback schedule on the final
  // straight, not an endpoint-pointing manoeuvre.
  bool terminal_alignment_enable_{true};
  bool terminal_alignment_speed_limit_enable_{false};
  double terminal_alignment_start_distance_{1.20};
  double terminal_alignment_lateral_activation_{0.008};
  double terminal_alignment_heading_activation_{0.015};
  double terminal_alignment_max_speed_{1.00};
  double terminal_alignment_min_speed_{1.00};
  double terminal_alignment_feedback_scale_{0.95};

  // Reverse uses a slightly earlier and slower final-straight schedule because
  // the measured reverse run otherwise reaches the terminal tangent plane with
  // 11-18 mm residual lateral error.
  double reverse_terminal_alignment_start_distance_{1.60};
  double reverse_terminal_alignment_max_speed_{1.00};
  double reverse_terminal_alignment_min_speed_{1.00};
  double reverse_terminal_alignment_feedback_scale_{0.90};
  double reverse_terminal_longitudinal_speed_gain_{0.50};
  double reverse_terminal_correction_min_velocity_{0.012};
  double reverse_terminal_crawl_min_along_distance_{0.002};
  double reverse_terminal_crawl_max_lateral_error_{0.010};

  // Direction-specific, low-amplitude terminal lateral convergence.
  bool forward_terminal_lateral_convergence_enable_{true};
  double forward_terminal_lateral_convergence_start_distance_{1.20};
  double forward_terminal_lateral_convergence_fade_distance_{0.030};
  double forward_terminal_lateral_convergence_deadband_{0.004};
  double forward_terminal_lateral_convergence_gain_{1.00};
  double forward_terminal_lateral_convergence_max_steering_{0.025};

  // Reverse low-speed terminal lateral convergence.
  bool reverse_terminal_lateral_convergence_enable_{true};
  double reverse_terminal_lateral_convergence_start_distance_{1.60};
  double reverse_terminal_lateral_convergence_fade_distance_{0.012};
  double reverse_terminal_lateral_convergence_deadband_{0.003};
  double reverse_terminal_lateral_convergence_gain_{1.05};
  double reverse_terminal_lateral_convergence_max_steering_{0.028};

  // Curvature and command smoothing.
  bool curvature_smoothing_enable_{true};
  double curvature_smoothing_alpha_{0.55};
  double curve_entry_smoothing_alpha_{0.85};
  double curve_exit_smoothing_alpha_{0.82};
  double curve_entry_rate_multiplier_{1.40};
  double max_curvature_rate_{3.00};

  bool empirical_yaw_dynamics_enable_{true};
  double empirical_max_yaw_acceleration_{0.60};
  double empirical_sign_change_max_yaw_acceleration_{0.35};
  double empirical_terminal_max_yaw_acceleration_{0.25};
  double empirical_terminal_yaw_accel_distance_{0.60};
  double empirical_curvature_rate_speed_floor_{0.10};

  double straight_sign_change_smoothing_alpha_{0.30};
  double straight_sign_change_rate_multiplier_{0.45};
  double straight_output_curvature_deadband_{0.006};
  double straight_output_deadband_lateral_error_{0.006};
  double straight_output_deadband_heading_error_{0.012};
  double straight_sign_reversal_min_curvature_{0.025};
  double control_period_{0.10};

  bool command_smoothing_enable_{true};
  double command_smoothing_alpha_{0.65};
  double command_deceleration_alpha_{0.65};

  // Once the final 2 m approach begins, the speed envelope is only allowed to
  // decrease. This removes speed up/down oscillations caused by centimetre-scale
  // localization and curvature-preview fluctuations while preserving 1 m/s
  // cruise before the terminal approach.
  bool terminal_monotonic_speed_enable_{true};
  double terminal_monotonic_release_distance_{2.20};

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

  bool post_curve_was_curved_{false};
  double post_curve_recovery_remaining_{0.0};
  int post_curve_direction_{0};

  bool reverse_straight_stabilization_active_{false};
  int reverse_straight_stabilization_direction_{0};

  int terminal_hold_count_{0};
  bool terminal_stop_latched_{false};
  bool terminal_stop_result_printed_{false};
  bool terminal_speed_profile_active_{false};
  int terminal_speed_profile_direction_{0};
  double terminal_speed_profile_cap_{1.0};

  ControlDebug control_debug_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_pub_;
};

}  // namespace lqr

#endif  // LQR__LQR_HPP_

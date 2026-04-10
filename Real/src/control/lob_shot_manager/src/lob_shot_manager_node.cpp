#include "lob_shot_manager/lob_shot_manager_node.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "logger.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "projectile_motion/gaf_projectile_solver.hpp"

namespace lob_shot_manager
{

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace

LobShotManagerNode::LobShotManagerNode(const rclcpp::NodeOptions & options)
: Node("lob_shot_manager", options)
{
  // Declare parameters
  this->declare_parameter("target_x", 0.0);
  this->declare_parameter("target_y", 0.0);
  this->declare_parameter("target_z", 0.0);
  this->declare_parameter("bullet_speed", 16.0);
  this->declare_parameter("friction_coeff", 0.001);
  this->declare_parameter("yaw_tolerance", 0.02);
  this->declare_parameter("pitch_tolerance", 0.02);
  this->declare_parameter("pitch_converge_threshold", 0.005);
  this->declare_parameter("max_pitch_iters", 5);
  this->declare_parameter("tf_timeout", 5.0);
  this->declare_parameter("aim_timeout", 5.0);
  this->declare_parameter("gimbal_cmd_topic", "cmd_gimbal_joint");
  this->declare_parameter("joint_state_topic", "joint_states");
  this->declare_parameter("gimbal_feedback_topic", "gimbal_world_feedback");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("chassis_frame", "chassis");
  this->declare_parameter("muzzle_frame", "muzzle");
  this->declare_parameter("yaw_joint_name", "gimbal_yaw_joint");
  this->declare_parameter("pitch_joint_name", "gimbal_pitch_joint");
  this->declare_parameter("gimbal_yaw_frame", "gimbal_yaw");
  this->declare_parameter("offset_min_wait", 2.0);

  // Load parameters
  target_x_ = this->get_parameter("target_x").as_double();
  target_y_ = this->get_parameter("target_y").as_double();
  target_z_ = this->get_parameter("target_z").as_double();
  bullet_speed_ = this->get_parameter("bullet_speed").as_double();
  friction_coeff_ = this->get_parameter("friction_coeff").as_double();
  yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
  pitch_tolerance_ = this->get_parameter("pitch_tolerance").as_double();
  pitch_converge_threshold_ = this->get_parameter("pitch_converge_threshold").as_double();
  max_pitch_iters_ = this->get_parameter("max_pitch_iters").as_int();
  tf_timeout_ = this->get_parameter("tf_timeout").as_double();
  aim_timeout_ = this->get_parameter("aim_timeout").as_double();
  gimbal_cmd_topic_ = this->get_parameter("gimbal_cmd_topic").as_string();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  gimbal_world_topic_ = this->get_parameter("gimbal_feedback_topic").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  chassis_frame_ = this->get_parameter("chassis_frame").as_string();
  muzzle_frame_ = this->get_parameter("muzzle_frame").as_string();
  yaw_joint_name_ = this->get_parameter("yaw_joint_name").as_string();
  pitch_joint_name_ = this->get_parameter("pitch_joint_name").as_string();
  gimbal_yaw_frame_ = this->get_parameter("gimbal_yaw_frame").as_string();
  offset_min_wait_ = this->get_parameter("offset_min_wait").as_double();

  node_start_time_ = this->now();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Aiming library
  aiming_ = std::make_shared<lob_shot_aiming::LobShotAiming>(
    tf_buffer_, map_frame_, chassis_frame_, muzzle_frame_);

  // Subscribers
  trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "lob_shot/trigger", 10,
    std::bind(&LobShotManagerNode::triggerCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10,
    std::bind(&LobShotManagerNode::jointStateCallback, this, std::placeholders::_1));

  gimbal_world_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    gimbal_world_topic_, 10,
    std::bind(&LobShotManagerNode::gimbalWorldCallback, this, std::placeholders::_1));

  // Publishers
  gimbal_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(gimbal_cmd_topic_, 10);
  ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("lob_shot/ready_to_shoot", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("lob_shot/status", 10);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lob_shot/visualization", 10);

  // Main loop timer at 50Hz
  tick_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LobShotManagerNode::tick, this));

  state_enter_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "Lob shot manager initialized. Target: (%.2f, %.2f, %.2f), speed: %.1f m/s",
    target_x_, target_y_, target_z_, bullet_speed_);
}

void LobShotManagerNode::tick()
{
  // Handle cancel request (from trigger data=false)
  if (cancel_requested_) {
    cancel_requested_ = false;
    triggered_ = false;
    if (state_ != State::IDLE) {
      RCLCPP_INFO(this->get_logger(), "Lob shot cancelled");
      publishStatus("Cancelled");
      setState(State::IDLE);
    }
  }

  switch (state_) {
    case State::IDLE:            handleIdle(); break;
    case State::WAITING_TF:      handleWaitingTf(); break;
    case State::YAW_AIMING:      handleYawAiming(); break;
    case State::YAW_CONVERGING:  handleYawConverging(); break;
    case State::PITCH_AIMING:    handlePitchAiming(); break;
    case State::PITCH_CONVERGING:handlePitchConverging(); break;
    case State::READY:           handleReady(); break;
  }
  publishVisualization();
}

// --- State handlers ---

void LobShotManagerNode::handleIdle()
{
  // Publish ready = false
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = false;
  ready_pub_->publish(ready_msg);
  clearVisualization();

  if (triggered_) {
    triggered_ = false;
    RCLCPP_INFO(this->get_logger(), "Lob shot triggered!");
    publishStatus("Triggered, waiting for TF...");
    setState(State::WAITING_TF);
  }
}

void LobShotManagerNode::handleWaitingTf()
{
  if (aiming_->isTfReady()) {
    publishStatus("TF ready, solving yaw...");
    setState(State::YAW_AIMING);
    return;
  }

  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > tf_timeout_) {
    RCLCPP_ERROR(this->get_logger(), "TF timeout (%.1fs), aborting", tf_timeout_);
    publishStatus("ERROR: TF timeout");
    setState(State::IDLE);
  }
}

void LobShotManagerNode::handleYawAiming()
{
  if (!tryComputeYawOffset()) {
    publishStatus("Waiting yaw offset calibration...");
    return;
  }

  auto result = aiming_->solveYaw(target_x_, target_y_, target_z_);
  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Yaw solve failed: %s", result.message.c_str());
    publishStatus("ERROR: " + result.message);
    setState(State::IDLE);
    return;
  }

  target_yaw_ = result.angle;
  target_yaw_in_map_ = result.angle_world;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1)
      << "Yaw solved: " << (target_yaw_ * 180.0 / M_PI) << " deg (map: "
      << (target_yaw_in_map_ * 180.0 / M_PI) << " deg, offset: "
      << (yaw_offset_ * 180.0 / M_PI) << " deg), waiting convergence...";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  utils::logger()->info(
    "[LOB_CMD] yaw solved: target_rel={:.4f} target_map={:.4f} yaw_offset={:.4f}",
    target_yaw_, target_yaw_in_map_, yaw_offset_);
  publishStatus(oss.str());

  setState(State::YAW_CONVERGING);
}

void LobShotManagerNode::handleYawConverging()
{
  const double target_yaw_in_imu = normalizeAngle(target_yaw_in_map_ - yaw_offset_);
  publishGimbalCmd(target_yaw_in_imu, current_pitch_);

  const double current_yaw_in_map = normalizeAngle(world_yaw_ + yaw_offset_);
  const double yaw_error = normalizeAngle(current_yaw_in_map - target_yaw_in_map_);

  if (fabs(yaw_error) < yaw_tolerance_) {
    utils::logger()->info(
      "[LOB_AIM] yaw converged: cur_map={:.4f} target_map={:.4f} cur_imu={:.4f} cmd_imu={:.4f} err={:.4f}",
      current_yaw_in_map, target_yaw_in_map_, world_yaw_, target_yaw_in_imu, yaw_error);
    publishStatus("Yaw converged, solving pitch...");
    pitch_iter_count_ = 0;
    setState(State::PITCH_AIMING);
    return;
  }

  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > aim_timeout_) {
    RCLCPP_ERROR(this->get_logger(),
      "Yaw convergence timeout. map_error=%.3f rad, cur_map=%.3f, target_map=%.3f, yaw_offset=%.3f",
      yaw_error, current_yaw_in_map, target_yaw_in_map_, yaw_offset_);
    utils::logger()->warn(
      "[LOB_AIM] yaw timeout: cur_map={:.4f} target_map={:.4f} cur_imu={:.4f} cmd_imu={:.4f} offset={:.4f} err={:.4f}",
      current_yaw_in_map, target_yaw_in_map_, world_yaw_, target_yaw_in_imu, yaw_offset_, yaw_error);
    publishStatus("ERROR: Yaw convergence timeout");
    setState(State::IDLE);
  }
}

void LobShotManagerNode::handlePitchAiming()
{
  auto result = aiming_->solvePitch(
    target_x_, target_y_, target_z_, bullet_speed_, friction_coeff_);
  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Pitch solve failed: %s", result.message.c_str());
    publishStatus("ERROR: " + result.message);
    setState(State::IDLE);
    return;
  }

  prev_pitch_ = target_pitch_;
  target_pitch_ = result.angle;
  last_horizontal_dist_ = result.horizontal_dist;
  last_height_diff_ = result.height_diff;
  pitch_iter_count_++;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "Pitch solved (iter " << pitch_iter_count_ << "): "
      << (target_pitch_ * 180.0 / M_PI) << " deg, dist="
      << result.horizontal_dist << "m, dh=" << result.height_diff << "m";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  publishStatus(oss.str());

  setState(State::PITCH_CONVERGING);
}

void LobShotManagerNode::handlePitchConverging()
{
  const double target_yaw_in_imu = normalizeAngle(target_yaw_in_map_ - yaw_offset_);
  publishGimbalCmd(target_yaw_in_imu, target_pitch_);

  double pitch_error = fabs(world_pitch_ - target_pitch_);
  if (pitch_error < pitch_tolerance_) {
    // Joint reached target, check if outer iteration is needed
    double pitch_change = fabs(target_pitch_ - prev_pitch_);
    bool first_iter = (pitch_iter_count_ == 1);

    if (!first_iter && pitch_change < pitch_converge_threshold_) {
      // Converged
      publishStatus("Pitch converged, READY TO SHOOT!");
      setState(State::READY);
      return;
    }

    if (pitch_iter_count_ >= max_pitch_iters_) {
      RCLCPP_WARN(this->get_logger(),
        "Max pitch iterations (%d) reached, pitch_change=%.4f rad. Proceeding anyway.",
        max_pitch_iters_, pitch_change);
      publishStatus("Pitch max iters reached, READY TO SHOOT!");
      setState(State::READY);
      return;
    }

    // Re-solve with updated muzzle position
    publishStatus("Pitch re-solving (outer iteration)...");
    setState(State::PITCH_AIMING);
    return;
  }

  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > aim_timeout_) {
    RCLCPP_ERROR(this->get_logger(),
      "Pitch convergence timeout. Current: %.3f, target: %.3f, error: %.4f rad",
      world_pitch_, target_pitch_, pitch_error);
    publishStatus("ERROR: Pitch convergence timeout");
    setState(State::IDLE);
  }
}

void LobShotManagerNode::handleReady()
{
  // Keep publishing ready + gimbal command to hold position
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  ready_pub_->publish(ready_msg);

  const double target_yaw_in_imu = normalizeAngle(target_yaw_in_map_ - yaw_offset_);
  publishGimbalCmd(target_yaw_in_imu, target_pitch_);

  // If re-triggered, restart aiming directly (skip IDLE)
  if (triggered_) {
    triggered_ = false;
    RCLCPP_INFO(this->get_logger(), "Re-triggered, restarting aiming...");
    publishStatus("Re-triggered, solving yaw...");
    setState(State::WAITING_TF);
  }
}

// --- Helpers ---

void LobShotManagerNode::setState(State new_state)
{
  RCLCPP_DEBUG(this->get_logger(), "State: %s -> %s",
    stateName(state_).c_str(), stateName(new_state).c_str());
  state_ = new_state;
  state_enter_time_ = this->now();
}

void LobShotManagerNode::publishGimbalCmd(double yaw, double pitch)
{
  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = this->now();
  cmd.name = {yaw_joint_name_, pitch_joint_name_};
  cmd.position = {yaw, pitch};
  gimbal_cmd_pub_->publish(cmd);
}

void LobShotManagerNode::publishStatus(const std::string & msg)
{
  std_msgs::msg::String status;
  status.data = "[" + stateName(state_) + "] " + msg;
  status_pub_->publish(status);
}

bool LobShotManagerNode::tryComputeYawOffset()
{
  if (yaw_offset_ready_) {
    return true;
  }

  static int wait_log_count = 0;
  const double elapsed_since_start = (this->now() - node_start_time_).seconds();
  if (elapsed_since_start < offset_min_wait_) {
    if (wait_log_count < 5) {
      utils::logger()->info(
        "[LOB_YAW_OFFSET] waiting startup: elapsed={:.3f}s min_wait={:.3f}s",
        elapsed_since_start, offset_min_wait_);
      ++wait_log_count;
    }
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_map_gimbal;
  try {
    tf_map_gimbal = tf_buffer_->lookupTransform(
      map_frame_, gimbal_yaw_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    if (wait_log_count < 10) {
      utils::logger()->warn(
        "[LOB_YAW_OFFSET] waiting TF {}->{}: {}",
        map_frame_, gimbal_yaw_frame_, ex.what());
      ++wait_log_count;
    }
    return false;
  }

  const double map_yaw = tf2::getYaw(tf_map_gimbal.transform.rotation);
  const double imu_yaw = world_yaw_;
  yaw_offset_ = normalizeAngle(map_yaw - imu_yaw);
  yaw_offset_ready_ = true;

  tf2::Quaternion q;
  tf2::fromMsg(tf_map_gimbal.transform.rotation, q);
  double map_roll = 0.0;
  double map_pitch = 0.0;
  double map_yaw_check = 0.0;
  tf2::Matrix3x3(q).getRPY(map_roll, map_pitch, map_yaw_check);
  const double pitch_diff = normalizeAngle(map_pitch - world_pitch_);

  utils::logger()->info(
    "[LOB_YAW_OFFSET] computed: map_yaw={:.4f} imu_yaw={:.4f} offset={:.4f}",
    map_yaw, imu_yaw, yaw_offset_);
  utils::logger()->info(
    "[LOB_PITCH_DIAG] map_pitch={:.4f} imu_pitch={:.4f} diff={:.4f} map_roll={:.4f}",
    map_pitch, world_pitch_, pitch_diff, map_roll);
  RCLCPP_INFO(this->get_logger(),
    "Yaw offset computed: map_yaw=%.4f imu_yaw=%.4f offset=%.4f rad",
    map_yaw, imu_yaw, yaw_offset_);
  RCLCPP_INFO(this->get_logger(),
    "Pitch diag: map_pitch=%.4f imu_pitch=%.4f diff=%.4f rad",
    map_pitch, world_pitch_, pitch_diff);

  return true;
}

std::string LobShotManagerNode::stateName(State s)
{
  switch (s) {
    case State::IDLE:             return "IDLE";
    case State::WAITING_TF:      return "WAITING_TF";
    case State::YAW_AIMING:      return "YAW_AIMING";
    case State::YAW_CONVERGING:  return "YAW_CONVERGING";
    case State::PITCH_AIMING:    return "PITCH_AIMING";
    case State::PITCH_CONVERGING:return "PITCH_CONVERGING";
    case State::READY:           return "READY";
    default:                     return "UNKNOWN";
  }
}

// --- Visualization ---

void LobShotManagerNode::clearVisualization()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = map_frame_;
  del.header.stamp = this->now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.ns = "lob_shot";
  markers.markers.push_back(del);
  viz_pub_->publish(markers);
}

void LobShotManagerNode::publishVisualization()
{
  if (state_ == State::IDLE || state_ == State::WAITING_TF) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  auto stamp = this->now();

  // --- Marker 0: 目标点 (红色球) ---
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "lob_shot";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = target_x_;
    m.pose.position.y = target_y_;
    m.pose.position.z = target_z_;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0;
    m.lifetime = rclcpp::Duration(0, 0);
    markers.markers.push_back(m);
  }

  // 查 TF: gimbal_yaw 在 map 下的位姿 (用于 yaw 可视化)
  geometry_msgs::msg::TransformStamped tf_map_chassis;
  bool have_chassis_tf = false;
  try {
    tf_map_chassis = tf_buffer_->lookupTransform(map_frame_, chassis_frame_, tf2::TimePointZero);
    have_chassis_tf = true;
  } catch (const tf2::TransformException &) {}

  if (!have_chassis_tf) {
    viz_pub_->publish(markers);
    return;
  }

  double gimbal_x = tf_map_chassis.transform.translation.x;
  double gimbal_y = tf_map_chassis.transform.translation.y;
  double gimbal_z = tf_map_chassis.transform.translation.z;

  // --- Marker 1: 云台到目标连线 (绿色虚线) ---
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "lob_shot";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.02;
    m.color.r = 0.2; m.color.g = 1.0; m.color.b = 0.2; m.color.a = 0.8;

    geometry_msgs::msg::Point p1, p2;
    p1.x = gimbal_x; p1.y = gimbal_y; p1.z = gimbal_z + 0.3;
    p2.x = target_x_; p2.y = target_y_; p2.z = gimbal_z + 0.3;
    m.points.push_back(p1);
    m.points.push_back(p2);
    markers.markers.push_back(m);
  }

  // --- Marker 2: 当前云台朝向射线 (黄→绿, yaw阶段变色) ---
  {
    // 直接使用 IMU 世界系反馈 world_yaw_，避免小陀螺时 TF chassis_yaw 延迟
    // 导致 (chassis_yaw_stale + current_yaw_live) 产生高频抖动
    double ray_len = std::hypot(target_x_ - gimbal_x, target_y_ - gimbal_y);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "lob_shot";
    m.id = 2;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    const double current_yaw_in_map = normalizeAngle(world_yaw_ + yaw_offset_);

    geometry_msgs::msg::Point start, end;
    start.x = gimbal_x; start.y = gimbal_y; start.z = gimbal_z + 0.3;
    end.x = gimbal_x + ray_len * cos(current_yaw_in_map);
    end.y = gimbal_y + ray_len * sin(current_yaw_in_map);
    end.z = gimbal_z + 0.3;
    m.points.push_back(start);
    m.points.push_back(end);

    m.scale.x = 0.03;  // shaft diameter
    m.scale.y = 0.06;  // head diameter
    m.scale.z = 0.08;  // head length

    // yaw 对齐时从黄变绿 (统一在 map 坐标系比较)
    double yaw_err = fabs(normalizeAngle(current_yaw_in_map - target_yaw_in_map_));
    double blend = std::min(1.0, yaw_err / 0.5);  // 0.5rad内线性过渡
    m.color.r = blend;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    markers.markers.push_back(m);
  }

  // --- Marker 3: 弹道轨迹弧线 (仅 pitch 阶段) ---
  bool have_trajectory = (state_ == State::PITCH_AIMING ||
                          state_ == State::PITCH_CONVERGING ||
                          state_ == State::READY);
  if (have_trajectory && last_horizontal_dist_ > 0.1) {
    // solver 输出的 pitch angle (正=向上), target_pitch_ 是 joint 角度(负=向上)
    double launch_angle = -target_pitch_;

    projectile_motion::GafProjectileSolver solver(bullet_speed_, friction_coeff_);
    auto traj_2d = solver.computeTrajectory(launch_angle, last_horizontal_dist_, 60);

    // 2D → 3D: 需要 muzzle 位置和朝向目标的方向
    geometry_msgs::msg::TransformStamped tf_map_muzzle;
    bool have_muzzle_tf = false;
    try {
      tf_map_muzzle = tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
      have_muzzle_tf = true;
    } catch (const tf2::TransformException &) {}

    if (have_muzzle_tf) {
      double mx = tf_map_muzzle.transform.translation.x;
      double my = tf_map_muzzle.transform.translation.y;
      double mz = tf_map_muzzle.transform.translation.z;

      double dx = target_x_ - mx;
      double dy = target_y_ - my;
      double norm_xy = std::hypot(dx, dy);
      double dir_x = (norm_xy > 0.01) ? dx / norm_xy : 1.0;
      double dir_y = (norm_xy > 0.01) ? dy / norm_xy : 0.0;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = "lob_shot";
      m.id = 3;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.03;

      // READY=青色, 迭代中=橙色
      if (state_ == State::READY) {
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      } else {
        m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0; m.color.a = 0.9;
      }

      for (auto & [horiz, height] : traj_2d) {
        geometry_msgs::msg::Point p;
        p.x = mx + horiz * dir_x;
        p.y = my + horiz * dir_y;
        p.z = mz + height;
        m.points.push_back(p);
      }
      markers.markers.push_back(m);

      // --- Marker 4: 预计落点 (轨迹终点, 小球) ---
      if (!traj_2d.empty()) {
        auto & [last_x, last_h] = traj_2d.back();
        visualization_msgs::msg::Marker mp;
        mp.header.frame_id = map_frame_;
        mp.header.stamp = stamp;
        mp.ns = "lob_shot";
        mp.id = 4;
        mp.type = visualization_msgs::msg::Marker::SPHERE;
        mp.action = visualization_msgs::msg::Marker::ADD;
        mp.pose.position.x = mx + last_x * dir_x;
        mp.pose.position.y = my + last_x * dir_y;
        mp.pose.position.z = mz + last_h;
        mp.pose.orientation.w = 1.0;
        mp.scale.x = mp.scale.y = mp.scale.z = 0.1;

        if (state_ == State::READY) {
          mp.color.r = 0.0; mp.color.g = 1.0; mp.color.b = 0.0; mp.color.a = 1.0;
        } else {
          mp.color.r = 1.0; mp.color.g = 0.6; mp.color.b = 0.0; mp.color.a = 1.0;
        }
        markers.markers.push_back(mp);
      }
    }
  }

  viz_pub_->publish(markers);
}

// --- Callbacks ---

void LobShotManagerNode::triggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    triggered_ = true;
  } else {
    cancel_requested_ = true;
  }
}

void LobShotManagerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == yaw_joint_name_) {
      current_yaw_ = msg->position[i];
    } else if (msg->name[i] == pitch_joint_name_) {
      current_pitch_ = msg->position[i];
    }
  }
}

void LobShotManagerNode::gimbalWorldCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == yaw_joint_name_) {
      world_yaw_ = msg->position[i];
    } else if (msg->name[i] == pitch_joint_name_) {
      world_pitch_ = msg->position[i];
    }
  }
}

}  // namespace lob_shot_manager

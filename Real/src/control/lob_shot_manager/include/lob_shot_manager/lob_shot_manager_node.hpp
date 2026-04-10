#ifndef LOB_SHOT_MANAGER__LOB_SHOT_MANAGER_NODE_HPP_
#define LOB_SHOT_MANAGER__LOB_SHOT_MANAGER_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "lob_shot_aiming/lob_shot_aiming.hpp"

namespace lob_shot_manager
{

enum class State
{
  IDLE,
  WAITING_TF,
  YAW_AIMING,
  YAW_CONVERGING,
  PITCH_AIMING,
  PITCH_CONVERGING,
  READY
};

class LobShotManagerNode : public rclcpp::Node
{
public:
  explicit LobShotManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Timer callback (main loop)
  void tick();

  // State handlers
  void handleIdle();
  void handleWaitingTf();
  void handleYawAiming();
  void handleYawConverging();
  void handlePitchAiming();
  void handlePitchConverging();
  void handleReady();

  // Helpers
  void setState(State new_state);
  void publishGimbalCmd(double yaw, double pitch);
  void publishStatus(const std::string & msg);
  void publishVisualization();
  void clearVisualization();
  std::string stateName(State s);

  // Yaw offset calibration (IMU世界系 → map帧的 yaw 补偿)
  bool tryComputeYawOffset();

  // Callbacks
  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void gimbalWorldCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gimbal_world_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Aiming library
  std::shared_ptr<lob_shot_aiming::LobShotAiming> aiming_;

  // State machine
  State state_{State::IDLE};
  bool triggered_{false};
  bool cancel_requested_{false};
  rclcpp::Time state_enter_time_;

  // Current joint readings
  double current_yaw_{0.0};
  double current_pitch_{0.0};

  // IMU反馈的角度
  double world_yaw_{0.0};
  double world_pitch_{0.0};

  // Yaw offset: map帧 和 IMU世界系 之间的 yaw 差值
  // offset = map_yaw_of_gimbal - imu_yaw_of_gimbal
  // 发给云台 PID 的指令: target_in_imu = target_in_map - yaw_offset_
  double yaw_offset_{0.0};
  bool yaw_offset_ready_{false};
  rclcpp::Time node_start_time_;

  // 计算目标
  double target_yaw_{0.0};
  double target_yaw_in_map_{0.0};  // world-frame yaw (for sim PID command)
  double target_pitch_{0.0};
  double prev_pitch_{0.0};     // for outer iteration convergence check
  int pitch_iter_count_{0};
  double last_horizontal_dist_{0.0};
  double last_height_diff_{0.0};

  // Parameters
  double target_x_, target_y_, target_z_;
  double bullet_speed_;
  double friction_coeff_;
  double yaw_tolerance_;
  double pitch_tolerance_;
  double pitch_converge_threshold_;
  int max_pitch_iters_;
  double tf_timeout_;
  double aim_timeout_;
  std::string gimbal_cmd_topic_;
  std::string joint_state_topic_;
  std::string gimbal_world_topic_;
  std::string map_frame_;
  std::string chassis_frame_;
  std::string muzzle_frame_;
  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  std::string gimbal_yaw_frame_;   // 云台 yaw 轴 TF 帧名，用于计算 offset
  double offset_min_wait_;         // 启动后等待 relocalization 稳定的最短时间 (s)
};

}  // namespace lob_shot_manager

#endif  // LOB_SHOT_MANAGER__LOB_SHOT_MANAGER_NODE_HPP_

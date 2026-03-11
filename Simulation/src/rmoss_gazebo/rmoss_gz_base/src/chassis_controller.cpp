// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "rmoss_gz_base/chassis_controller.hpp"

#include <memory>
#include <string>


namespace rmoss_gz_base
{
ChassisController::ChassisController(
  rclcpp::Node::SharedPtr node,
  Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator,
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_encoder,
  const std::string & controller_name)
: node_(node), chassis_actuator_(chassis_actuator), gimbal_encoder_(gimbal_encoder)
{
  node_->declare_parameter(controller_name + ".follow_yaw", follow_mode_flag_);
  node_->get_parameter(controller_name + ".follow_yaw", follow_mode_flag_);
  declare_pid_parameter(node_, controller_name + ".follow_yaw_pid");
  get_pid_parameter(node_, controller_name + ".follow_yaw_pid", chassis_pid_param_);
  set_chassis_pid(chassis_pid_param_);
  // sensor data
  gimbal_encoder_->add_callback(
    [this](const rmoss_interfaces::msg::Gimbal & data, const rclcpp::Time & /*stamp*/) {
      // Limit yaw data to [0, 2*PI)
      cur_yaw_ = std::remainder(data.yaw, 2 * M_PI);
    });
  // ros sub
  using namespace std::placeholders;
  ros_chassis_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
    "robot_base/chassis_cmd", 10, std::bind(&ChassisController::chassis_cb, this, _1));
  ros_cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&ChassisController::cmd_vel_cb, this, _1));
  // timer and set_parameters callback
  auto period = std::chrono::milliseconds(10);
  controller_timer_ = node_->create_wall_timer(
    period, std::bind(&ChassisController::update, this));
}

void ChassisController::update()
{
  static const auto DT = std::chrono::milliseconds(10);
  geometry_msgs::msg::Twist result_vel;

  if (follow_mode_flag_) {
    // follow mode
    result_vel.linear = target_vel_.linear;
    result_vel.angular.z = chassis_pid_.Update(-cur_yaw_, DT);
  } else {
    result_vel.angular.z = target_vel_.angular.z;
    result_vel.linear.x =  target_vel_.linear.x * std::cos(-cur_yaw_) + target_vel_.linear.y * std::sin(-cur_yaw_);
    result_vel.linear.y = -target_vel_.linear.x * std::sin(-cur_yaw_) + target_vel_.linear.y * std::cos(-cur_yaw_);
  }
  // publish CMD
  chassis_actuator_->set(result_vel);
}

void ChassisController::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  if (msg->type == msg->VELOCITY) {
    target_vel_ = msg->twist;
    follow_mode_flag_ = false;
  } else if (msg->type == msg->FOLLOW_GIMBAL) {
    target_vel_.linear.x = msg->twist.linear.x;
    target_vel_.linear.y = msg->twist.linear.y;
    if (!follow_mode_flag_) {
      follow_mode_flag_ = true;
      chassis_pid_.Reset();
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "chassis type[%d] isn't supported!", msg->type);
  }
}

void ChassisController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_vel_ = *msg;
  follow_mode_flag_ = false;
}

void ChassisController::set_chassis_pid(PidParam pid_param)
{
  chassis_pid_.Init(
    pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
    pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}

void ChassisController::reset()
{
  target_vel_.linear.x = 0;
  target_vel_.linear.y = 0;
  target_vel_.angular.z = 0;
  chassis_pid_.Reset();
}

}  // namespace rmoss_gz_base

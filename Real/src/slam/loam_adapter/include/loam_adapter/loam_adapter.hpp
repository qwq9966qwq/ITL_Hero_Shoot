#ifndef LOAM_ADAPTER__LOAM_ADAPTER_HPP_
#define LOAM_ADAPTER__LOAM_ADAPTER_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace loam_adapter
{

class LoamAdapterNode : public rclcpp::Node
{
public:
  explicit LoamAdapterNode(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_topic_;
  std::string pcd_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  bool initialized_;
  tf2::Transform T_ext_0_;
  tf2::Transform T_init_;              // Point_LIO 首帧输出，用于消除重力对齐偏移
  tf2::Transform T_pcd_transform_;     // 缓存: T_ext_0_ × T_init_⁻¹，用于点云变换
};

}  // namespace loam_adapter

#endif  // LOAM_ADAPTER__LOAM_ADAPTER_HPP_

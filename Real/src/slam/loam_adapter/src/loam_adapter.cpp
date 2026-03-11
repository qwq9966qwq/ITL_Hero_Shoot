#include "loam_adapter/loam_adapter.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_adapter
{

LoamAdapterNode::LoamAdapterNode(const rclcpp::NodeOptions & options)
: Node("loam_adapter", options), initialized_(false)
{
  this->declare_parameter<std::string>("odom_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("pcd_topic", "cloud_registered");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");

  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("pcd_topic", pcd_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 5,
    std::bind(&LoamAdapterNode::odometryCallback, this, std::placeholders::_1));
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pcd_topic_, 5,
    std::bind(&LoamAdapterNode::pointCloudCallback, this, std::placeholders::_1));
}

void LoamAdapterNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // Step 1: 首次回调，缓存启动时刻的外参和 Point_LIO 初始输出
  if (!initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(1.0));
      tf2::fromMsg(tf_stamped.transform, T_ext_0_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Waiting for initial TF: %s", ex.what());
      return;
    }

    // 缓存 Point_LIO 首帧输出（包含重力对齐旋转）
    // 后续用 T_init_⁻¹ × T_pointlio 得到纯相对运动，消除重力对齐偏移
    tf2::fromMsg(msg->pose.pose, T_init_);

    // 预计算点云变换: camera_init → odom = T(base←lidar) × T(lidar(0)←camera_init)
    T_pcd_transform_ = T_ext_0_ * T_init_.inverse();

    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Initialized: extrinsic %s->%s cached, gravity offset compensated",
      base_frame_.c_str(), lidar_frame_.c_str());
  }

  // Step 2: 实时查询当前时刻的 base_footprint → front_mid360 (T_ext_t)
  //         云台旋转时这个变换是动态的
  tf2::Transform T_ext_t;
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    tf2::fromMsg(tf_stamped.transform, T_ext_t);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Dynamic TF lookup failed: %s", ex.what());
    return;
  }

  // Step 3: 提取 Point_LIO 输出: T(camera_init → aft_mapped)
  tf2::Transform T_pointlio;
  tf2::fromMsg(msg->pose.pose, T_pointlio);

  // Step 4: 计算相对运动（消除重力对齐偏移）
  // T_delta = T(lidar(0) → lidar(t))，纯运动量，无重力对齐旋转
  tf2::Transform T_delta = T_init_.inverse() * T_pointlio;

  // Step 5: T(odom → base_footprint) = T_ext_0 × T_delta × inv(T_ext_t)
  tf2::Transform T_odom_base = T_ext_0_ * T_delta * T_ext_t.inverse();

  // Step 5: 发布 TF: odom → base_footprint
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform = tf2::toMsg(T_odom_base);
  tf_broadcaster_->sendTransform(tf_msg);

  // Step 6: 发布 nav_msgs/Odometry
  nav_msgs::msg::Odometry odom_out;
  odom_out.header.stamp = msg->header.stamp;
  odom_out.header.frame_id = odom_frame_;
  odom_out.child_frame_id = base_frame_;

  const auto & origin = T_odom_base.getOrigin();
  odom_out.pose.pose.position.x = origin.x();
  odom_out.pose.pose.position.y = origin.y();
  odom_out.pose.pose.position.z = origin.z();
  odom_out.pose.pose.orientation = tf2::toMsg(T_odom_base.getRotation());

  odom_pub_->publish(odom_out);
}

void LoamAdapterNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!initialized_) return;

  // 点云在 camera_init 帧下（Point_LIO 的重力对齐世界帧）
  // 用预计算的 T_pcd_transform_ = T_ext_0 × T_init⁻¹ 变换到 odom 帧
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, T_pcd_transform_, *msg, *out);
  pcd_pub_->publish(*out);
}

}  // namespace loam_adapter

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_adapter::LoamAdapterNode)

#include "relocalization/relocalization.hpp"

#include <pcl/filters/voxel_grid.h>

#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace relocalization
{

RelocalizationNode::RelocalizationNode(const rclcpp::NodeOptions & options)
: Node("relocalization", options),
  has_scan_(false),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  // GICP parameters
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);

  // Frame IDs
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_footprint");
  this->declare_parameter("lidar_frame", "front_mid360");

  // Files and topics
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("input_scan_topic", "registered_scan");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("input_scan_topic", input_scan_topic_);
  this->get_parameter("init_pose", init_pose_);

  // 用 init_pose 参数初始化配准种子 [x, y, z, roll, pitch, yaw]
  if (init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 加载先验 PCD 地图（camera_init 帧，重力对齐，直接作为 map 帧使用）
  loadGlobalMap(prior_pcd_file_);

  // 对全局地图做降采样、协方差估计、构建 KdTree（只做一次）
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_INFO(this->get_logger(), "Global map preprocessed: %zu points after downsampling",
    target_->size());

  // 发布先验地图供 RViz 可视化（transient_local QoS，只发一次，后来的订阅者也能收到）
  auto map_qos = rclcpp::QoS(1).transient_local();
  prior_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", map_qos);

  pcl::PointCloud<pcl::PointXYZ>::Ptr visual_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(global_map_);
  vg.setLeafSize(global_leaf_size_, global_leaf_size_, global_leaf_size_);
  vg.filter(*visual_map);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*visual_map, map_msg);
  map_msg.header.frame_id = map_frame_;
  map_msg.header.stamp = this->now();
  prior_map_pub_->publish(map_msg);
  RCLCPP_INFO(this->get_logger(), "Prior map published: %zu points (leaf=%.2f)",
    visual_map->size(), global_leaf_size_);

  // 订阅来自 loam_adapter 的 odom 帧点云
  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_scan_topic_, 10,
    std::bind(&RelocalizationNode::scanCallback, this, std::placeholders::_1));

  // 订阅 RViz 的初始位姿（备用手动纠正）
  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&RelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // 配准定时器 2Hz（每 500ms 做一次 GICP）
  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RelocalizationNode::performRegistration, this));

  // TF 发布定时器 20Hz（保证 TF 树时刻有效）
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RelocalizationNode::publishTransform, this));
}

void RelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map: %zu points", global_map_->points.size());

  // PCD 是 Point_LIO 建图时保存的，坐标系是 camera_init（重力对齐，Z 轴朝上）。
  // 不需要额外变换——直接把 camera_init 当作 map 帧使用。
  // GICP 会计算 T(map ← odom) 来对齐实时扫描（odom 帧，同样重力对齐）和先验地图，
  // 结果近似为平面刚体变换（x, y, yaw），不会引入 roll/pitch 倾斜。
}

void RelocalizationNode::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  has_scan_ = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void RelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    return;
  }

  // 降采样 + 协方差估计 + 构建 KdTree
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  // GICP 配准
  // target = 先验地图（map 帧）, source = 累积扫描（odom 帧）
  // 结果 T_target_source = T(map ← odom)
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge");
  }

  accumulated_cloud_->clear();
}

void RelocalizationNode::publishTransform()
{
  // 等到至少收到一帧扫描再开始发布
  if (!has_scan_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  // 时间戳加 0.1s 前瞻，防止 TF 因为延迟过期
  tf_msg.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;

  const Eigen::Vector3d t = result_t_.translation();
  const Eigen::Quaterniond q(result_t_.rotation());

  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

void RelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose: [%.2f, %.2f, %.2f]",
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  // RViz "2D Pose Estimate" 给出的是 map → base_footprint 的位姿
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() << msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z;
  map_to_base.linear() = Eigen::Quaterniond(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
    .toRotationMatrix();

  try {
    // 查 TF: base_footprint ← odom（把 odom 帧的点变到 base_footprint 帧）
    // 这就是 odom→base_footprint 的逆变换
    auto tf = tf_buffer_->lookupTransform(base_frame_, odom_frame_, tf2::TimePointZero);
    Eigen::Isometry3d base_from_odom = tf2::transformToEigen(tf.transform);

    // map→odom = map→base_footprint × base_footprint←odom
    Eigen::Isometry3d map_to_odom = map_to_base * base_from_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    RCLCPP_INFO(this->get_logger(), "Initial pose applied, map->odom updated");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Initial pose TF lookup failed: %s", ex.what());
  }
}

}  // namespace relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(relocalization::RelocalizationNode)

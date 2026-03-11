#ifndef RELOCALIZATION__RELOCALIZATION_HPP_
#define RELOCALIZATION__RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace relocalization
{

class RelocalizationNode : public rclcpp::Node
{
public:
  explicit RelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  // GICP parameters
  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  std::vector<double> init_pose_;

  // Frame IDs
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string prior_pcd_file_;
  std::string input_scan_topic_;

  // State
  bool has_scan_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  // GICP registration
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  // Timers
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Prior map publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
};

}  // namespace relocalization

#endif  // RELOCALIZATION__RELOCALIZATION_HPP_

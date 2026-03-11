#include "lob_shot_aiming/lob_shot_aiming.hpp"

#include <cmath>

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace lob_shot_aiming
{

LobShotAiming::LobShotAiming(
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & map_frame,
  const std::string & chassis_frame,
  const std::string & muzzle_frame)
: tf_buffer_(tf_buffer),
  map_frame_(map_frame),
  chassis_frame_(chassis_frame),
  muzzle_frame_(muzzle_frame)
{
}

bool LobShotAiming::isTfReady()
{
  try {
    tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

AimResult LobShotAiming::solveYaw(double target_x, double target_y, double /*target_z*/)
{
  AimResult result;
  result.success = false;
  result.horizontal_dist = 0;
  result.height_diff = 0;
  result.angle_world = 0;

  // Get chassis pose in map frame
  geometry_msgs::msg::TransformStamped tf_map_chassis;
  try {
    tf_map_chassis = tf_buffer_->lookupTransform(map_frame_, chassis_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    result.message = std::string("TF lookup failed (map->chassis): ") + ex.what();
    return result;
  }

  // Target direction in map frame
  double chassis_x = tf_map_chassis.transform.translation.x;
  double chassis_y = tf_map_chassis.transform.translation.y;
  double dx = target_x - chassis_x;
  double dy = target_y - chassis_y;
  double target_yaw_in_map = atan2(dy, dx);

  // Current chassis yaw in map frame
  double chassis_yaw = tf2::getYaw(tf_map_chassis.transform.rotation);

  // Joint angle = target direction relative to chassis
  result.angle = target_yaw_in_map - chassis_yaw;
  result.angle_world = target_yaw_in_map;

  // Normalize to [-pi, pi]
  while (result.angle > M_PI) result.angle -= 2 * M_PI;
  while (result.angle < -M_PI) result.angle += 2 * M_PI;

  result.horizontal_dist = std::hypot(dx, dy);
  result.success = true;
  result.message = "yaw solved";
  return result;
}

AimResult LobShotAiming::solvePitch(
  double target_x, double target_y, double target_z,
  double bullet_speed, double friction_coeff)
{
  AimResult result;
  result.success = false;

  // Get muzzle pose in map frame
  geometry_msgs::msg::TransformStamped tf_map_muzzle;
  try {
    tf_map_muzzle = tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    result.message = std::string("TF lookup failed (map->muzzle): ") + ex.what();
    return result;
  }

  double muzzle_x = tf_map_muzzle.transform.translation.x;
  double muzzle_y = tf_map_muzzle.transform.translation.y;
  double muzzle_z = tf_map_muzzle.transform.translation.z;

  double dx = target_x - muzzle_x;
  double dy = target_y - muzzle_y;
  result.horizontal_dist = std::hypot(dx, dy);
  result.height_diff = target_z - muzzle_z;

  // Use GAF solver
  projectile_motion::GafProjectileSolver solver(bullet_speed, friction_coeff);

  double pitch_angle;
  if (!solver.solve(result.horizontal_dist, result.height_diff, pitch_angle)) {
    result.message = "Ballistic solver failed: " + solver.error_message();
    return result;
  }

  // solver 输出: 正值 = 向上抛射
  // gimbal_pitch_joint 轴=Y, 正值=低头, 负值=抬头 → 取负
  result.angle = -pitch_angle;
  result.success = true;
  result.message = "pitch solved";
  return result;
}

}  // namespace lob_shot_aiming

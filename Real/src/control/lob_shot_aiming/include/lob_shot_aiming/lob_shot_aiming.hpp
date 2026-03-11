#ifndef LOB_SHOT_AIMING__LOB_SHOT_AIMING_HPP_
#define LOB_SHOT_AIMING__LOB_SHOT_AIMING_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"
#include "projectile_motion/gaf_projectile_solver.hpp"
#include "projectile_motion/gravity_projectile_solver.hpp"

namespace lob_shot_aiming
{

struct AimResult
{
  double angle;           // joint angle (rad), chassis-relative for yaw
  double angle_world;     // world-frame angle (rad), yaw only (for sim PID command)
  double horizontal_dist; // horizontal distance to target (m), pitch only
  double height_diff;     // height difference (m), pitch only
  std::string message;
  bool success;
};

class LobShotAiming
{
public:
  LobShotAiming(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & map_frame = "map",
    const std::string & chassis_frame = "chassis",
    const std::string & muzzle_frame = "muzzle");

  // Check if TF chain map->muzzle is available
  bool isTfReady();

  // Solve yaw joint angle to point at target (in map frame)
  AimResult solveYaw(double target_x, double target_y, double target_z);

  // Solve pitch joint angle using ballistic model
  AimResult solvePitch(
    double target_x, double target_y, double target_z,
    double bullet_speed, double friction_coeff);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;
  std::string chassis_frame_;
  std::string muzzle_frame_;
};

}  // namespace lob_shot_aiming

#endif  // LOB_SHOT_AIMING__LOB_SHOT_AIMING_HPP_

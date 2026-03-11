#ifndef PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_
#define PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_

#include <string>

namespace projectile_motion
{

class ProjectileSolverInterface
{
public:
  virtual ~ProjectileSolverInterface() = default;
  // target_x: horizontal distance (m), target_h: height difference (m, positive = target above)
  // angle: output launch angle (rad, positive = upward)
  virtual bool solve(double target_x, double target_h, double & angle) = 0;
  virtual std::string error_message() = 0;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__PROJECTILE_SOLVER_INTERFACE_HPP_

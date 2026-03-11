#ifndef PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_
#define PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_

#include <memory>
#include <string>

#include "projectile_motion/projectile_solver_interface.hpp"
#include "projectile_motion/iterative_projectile_tool.hpp"

namespace projectile_motion
{

// Gravity-only projectile solver
class GravityProjectileSolver : public ProjectileSolverInterface
{
public:
  explicit GravityProjectileSolver(double initial_vel);

  void set_initial_vel(double vel) { initial_vel_ = vel; }
  bool solve(double target_x, double target_h, double & angle) override;
  std::string error_message() override;

private:
  std::shared_ptr<IterativeProjectileTool> iterative_tool_;
  double initial_vel_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__GRAVITY_PROJECTILE_SOLVER_HPP_

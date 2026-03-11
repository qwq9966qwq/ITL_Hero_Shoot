#ifndef PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_
#define PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "projectile_motion/projectile_solver_interface.hpp"
#include "projectile_motion/iterative_projectile_tool.hpp"

namespace projectile_motion
{

// Gravity and Air Friction projectile solver
class GafProjectileSolver : public ProjectileSolverInterface
{
public:
  GafProjectileSolver(double initial_vel, double friction_coeff);

  void set_initial_vel(double vel) { initial_vel_ = vel; }
  void set_friction_coeff(double friction_coeff) { friction_coeff_ = friction_coeff; }
  bool solve(double target_x, double target_h, double & angle) override;
  std::string error_message() override;

  // Generate trajectory points: returns vector of (horizontal_dist, height)
  // angle: launch angle (rad, positive=upward), max_x: horizontal range (m)
  std::vector<std::pair<double, double>> computeTrajectory(
    double angle, double max_x, int num_points = 50);

private:
  std::shared_ptr<IterativeProjectileTool> iterative_tool_;
  double initial_vel_;
  double friction_coeff_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__GAF_PROJECTILE_SOLVER_HPP_

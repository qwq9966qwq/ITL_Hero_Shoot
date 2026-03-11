#include "projectile_motion/gravity_projectile_solver.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace projectile_motion
{

static constexpr double GRAVITY = 9.7913;

GravityProjectileSolver::GravityProjectileSolver(double initial_vel)
: initial_vel_(initial_vel)
{
  auto forward_motion = [&](double given_angle, double given_x, double & h, double & t) {
      t = given_x / (initial_vel_ * cos(given_angle));
      h = initial_vel_ * sin(given_angle) * t - GRAVITY * t * t / 2;
    };

  iterative_tool_ = std::make_shared<IterativeProjectileTool>();
  iterative_tool_->set_forward_motion(forward_motion);
  iterative_tool_->set_max_iter(20);
}

bool GravityProjectileSolver::solve(double target_x, double target_h, double & angle)
{
  return iterative_tool_->solve(target_x, target_h, angle);
}

std::string GravityProjectileSolver::error_message()
{
  return iterative_tool_->error_message();
}

}  // namespace projectile_motion

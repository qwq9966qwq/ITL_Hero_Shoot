#include "projectile_motion/gaf_projectile_solver.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace projectile_motion
{

static constexpr double GRAVITY = 9.7913;

GafProjectileSolver::GafProjectileSolver(double initial_vel, double friction_coeff)
: initial_vel_(initial_vel), friction_coeff_(friction_coeff)
{
  // Forward motion model: gravity + air friction
  // Air friction on x-axis considered only during descent phase
  auto forward_motion = [&](double given_angle, double given_x, double & h, double & t) {
      double & v = initial_vel_;
      if (given_angle > 0.01) {  // ascending phase exists
        double t0 = v * sin(given_angle) / GRAVITY;        // time to apex
        double x0 = v * cos(given_angle) * t0;             // horizontal distance at apex
        double y0 = GRAVITY * t0 * t0 / 2;                 // apex height
        if (given_x < x0) {  // still ascending, pure parabolic
          t = given_x / (v * cos(given_angle));
          h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
        } else {  // ascending then descending with air friction
          double x1 = given_x - x0;
          double t1 = (exp(friction_coeff_ * x1) - 1) /
            (friction_coeff_ * v * cos(given_angle));
          t = t0 + t1;
          h = y0 - GRAVITY * t1 * t1 / 2;
        }
      } else {  // only descending
        t = (exp(friction_coeff_ * given_x) - 1) /
          (friction_coeff_ * v * cos(given_angle));
        h = v * sin(given_angle) * t - GRAVITY * t * t / 2;
      }
    };

  iterative_tool_ = std::make_shared<IterativeProjectileTool>();
  iterative_tool_->set_forward_motion(forward_motion);
  iterative_tool_->set_max_iter(100);
}

bool GafProjectileSolver::solve(double target_x, double target_h, double & angle)
{
  return iterative_tool_->solve(target_x, target_h, angle);
}

std::string GafProjectileSolver::error_message()
{
  return iterative_tool_->error_message();
}

std::vector<std::pair<double, double>> GafProjectileSolver::computeTrajectory(
  double angle, double max_x, int num_points)
{
  std::vector<std::pair<double, double>> points;
  if (num_points < 2) num_points = 2;
  points.reserve(num_points);

  double v = initial_vel_;
  double t0 = 0, x0 = 0, y0 = 0;
  bool has_ascent = (angle > 0.01);

  if (has_ascent) {
    t0 = v * sin(angle) / GRAVITY;
    x0 = v * cos(angle) * t0;
    y0 = GRAVITY * t0 * t0 / 2;
  }

  for (int i = 0; i < num_points; i++) {
    double x = max_x * i / (num_points - 1);
    double h, t;

    if (has_ascent) {
      if (x < x0) {  // ascending: pure parabolic
        t = x / (v * cos(angle));
        h = v * sin(angle) * t - GRAVITY * t * t / 2;
      } else {  // descending: with air friction
        double x1 = x - x0;
        double t1 = (exp(friction_coeff_ * x1) - 1) /
          (friction_coeff_ * v * cos(angle));
        t = t0 + t1;
        h = y0 - GRAVITY * t1 * t1 / 2;
      }
    } else {  // only descending
      t = (exp(friction_coeff_ * x) - 1) /
        (friction_coeff_ * v * cos(angle));
      h = v * sin(angle) * t - GRAVITY * t * t / 2;
    }

    points.emplace_back(x, h);
  }

  return points;
}

}  // namespace projectile_motion

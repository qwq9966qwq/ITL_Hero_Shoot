#include "projectile_motion/iterative_projectile_tool.hpp"

#include <cmath>

namespace projectile_motion
{

bool IterativeProjectileTool::solve(double target_x, double target_h, double & angle)
{
  double aimed_h = target_h;
  double dh = 0;
  double tmp_angle = 0;
  double h = 0;
  double t = 0;

  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = atan2(aimed_h, target_x);
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      error_message_ = "iterative angle out of range (-80, 80) deg";
      return false;
    }
    forward_motion_func_(tmp_angle, target_x, h, t);
    if (t > 10) {
      error_message_ = "flight time (" + std::to_string(t) + "s) too long";
      return false;
    }
    dh = target_h - h;
    aimed_h += dh;
    if (fabs(dh) < 0.001) {
      break;
    }
  }
  if (fabs(dh) > 0.01) {
    error_message_ = "height error (" + std::to_string(dh) + "m) too large, not converged";
    return false;
  }
  angle = tmp_angle;
  return true;
}

}  // namespace projectile_motion

#include "IMU_Processing.h"
#include "logger.hpp"

const bool time_list(PointType & x, PointType & y) { return (x.curvature < y.curvature); };

void ImuProcess::set_gyr_cov(const V3D & scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D & scaler) { cov_vel_scale = scaler; }

ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  RCLCPP_WARN(logger, "reset ImuProcess");
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;
  init_iter_num = 1;
  after_imu_init_ = false;

  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0), gravity_(1),
    -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  utils::logger()->info("[Z_DIAG_GRAVITY] Set_init输入: tmp_gravity=({:.4f}, {:.4f}, {:.4f}) |tmp_gravity|={:.4f}",
    tmp_gravity(0), tmp_gravity(1), tmp_gravity(2), tmp_gravity.norm());
  utils::logger()->info("[Z_DIAG_GRAVITY] 目标gravity_=({:.4f}, {:.4f}, {:.4f}) |gravity_|={:.4f}",
    gravity_(0), gravity_(1), gravity_(2), gravity_.norm());
  utils::logger()->info("[Z_DIAG_GRAVITY] align_norm={:.6f}, align_cos={:.6f}, 夹角={:.2f}度",
    align_norm, align_cos, acos(std::clamp(align_cos, -1.0, 1.0)) * 180.0 / M_PI);

  if (align_norm < 1e-6) {
    if (align_cos > 1e-6) {
      rot = Eye3d;
      utils::logger()->info("[Z_DIAG_GRAVITY] 重力已对齐, rot=单位矩阵");
    } else {
      rot = -Eye3d;
      utils::logger()->warn("[Z_DIAG_GRAVITY] 重力反向! rot=-单位矩阵, 这可能导致Z轴翻转!");
    }
  } else {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
    utils::logger()->info("[Z_DIAG_GRAVITY] 对齐旋转角 align_angle=({:.4f}, {:.4f}, {:.4f}) rad = ({:.2f}, {:.2f}, {:.2f}) deg",
      align_angle(0), align_angle(1), align_angle(2),
      align_angle(0) * 180.0 / M_PI, align_angle(1) * 180.0 / M_PI, align_angle(2) * 180.0 / M_PI);
  }

  // 输出最终旋转矩阵
  utils::logger()->info("[Z_DIAG_GRAVITY] rot_init=\n  [{:.6f}, {:.6f}, {:.6f}]\n  [{:.6f}, {:.6f}, {:.6f}]\n  [{:.6f}, {:.6f}, {:.6f}]",
    rot(0,0), rot(0,1), rot(0,2), rot(1,0), rot(1,1), rot(1,2), rot(2,0), rot(2,1), rot(2,2));
}

void ImuProcess::IMU_init(const MeasureGroup & meas, int & N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    utils::logger()->info("[Z_DIAG_INIT] 首帧IMU数据 acc=({:.4f}, {:.4f}, {:.4f}) gyr=({:.4f}, {:.4f}, {:.4f})",
      mean_acc(0), mean_acc(1), mean_acc(2), mean_gyr(0), mean_gyr(1), mean_gyr(2));
  }

  for (const auto & imu : meas.imu) {
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }

  // 初始化完成时输出累积的mean_acc
  if (N >= MAX_INI_COUNT) {
    utils::logger()->info("[Z_DIAG_INIT] IMU初始化进度 N={}, mean_acc=({:.4f}, {:.4f}, {:.4f}) |mean_acc|={:.4f}, mean_gyr=({:.6f}, {:.6f}, {:.6f})",
      N, mean_acc(0), mean_acc(1), mean_acc(2), mean_acc.norm(),
      mean_gyr(0), mean_gyr(1), mean_gyr(2));
  }
}

void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en) {
    if (meas.imu.empty()) return;

    if (imu_need_init_) {
      {
        /// The very first lidar frame
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        if (init_iter_num > MAX_INI_COUNT) {
          RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
          utils::logger()->info("[Z_DIAG_INIT] IMU初始化完成! 最终 mean_acc=({:.4f}, {:.4f}, {:.4f}) |mean_acc|={:.4f}",
            mean_acc(0), mean_acc(1), mean_acc(2), mean_acc.norm());
          utils::logger()->info("[Z_DIAG_INIT] 最终 mean_gyr=({:.6f}, {:.6f}, {:.6f})",
            mean_gyr(0), mean_gyr(1), mean_gyr(2));
        }
        // *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  } else {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
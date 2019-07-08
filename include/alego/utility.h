#ifndef _LOAM_UTILITY_
#define _LOAM_UTILITY_

#include <iostream>
#include <vector>
#include <queue>
#include <deque>
#include <string>
#include <array>
#include <thread>
#include <chrono>
#include <mutex>
#include <memory>
#include <limits>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "alego/cloud_info.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

#define RAD2ANGLE(x) ((x)*180.0 / M_PI)
#define ANGLE2RAD(x) ((x) / 180.0 * M_PI)

const int N_SCAN = 16;
const double ang_res_x = 0.09; // 10Hz: 0.18, 5Hz: 0.09
const double ang_res_y = 2.0;
const double scan_period = 0.2; // 10Hz: 0.1, 5Hz: 0.2

const int Horizon_SCAN = 360.0 / ang_res_x + 0.5;
const double ang_bottom = 15.0;
const int ground_scan_id = 10;
const double sensor_mount_ang = 0.; // 向下为正，向上为负

const double seg_alpha_x = ANGLE2RAD(ang_res_x);
const double seg_alpha_y = ANGLE2RAD(ang_res_y);
// 可以调调参数
const double seg_theta = 1.047;
const int seg_valid_point_num = 5;
const int seg_valid_line_num = 3;

// (use_imu && use_odom) == false
const bool use_imu = true;
const bool use_odom = false;
const int imu_queue_length = 200;
const int odom_queue_length = 1000;

const double nearest_feature_dist = 25.; // sqaured, 对应 5m

enum LaserType
{
  LSLIDAR_C16,
  RFANS_16M
};

LaserType laser_type = LaserType::RFANS_16M;

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class TicToc
{
public:
  TicToc()
  {
    tic();
  }
  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    return elapsed.count() * 1000; // ms
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

class CornerCostFunction : public ceres::SizedCostFunction<1, 6>
{
public:
  CornerCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl) : cp_(cp), lpj_(lpj), lpl_(lpl) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
  {
    Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameters[0][4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameters[0][3], Eigen::Vector3d::UnitX())) * cp_ + Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);
    double k = std::sqrt(std::pow(lpj_.x() - lpl_.x(), 2) + std::pow(lpj_.y() - lpl_.y(), 2) + std::pow(lpj_.z() - lpl_.z(), 2));
    double a = (lp.y() - lpj_.y()) * (lp.z() - lpl_.z()) - (lp.z() - lpj_.z()) * (lp.y() - lpl_.y());
    double b = (lp.z() - lpj_.z()) * (lp.x() - lpl_.x()) - (lp.x() - lpj_.x()) * (lp.z() - lpl_.z());
    double c = (lp.x() - lpj_.x()) * (lp.y() - lpl_.y()) - (lp.y() - lpj_.y()) * (lp.x() - lpl_.x());
    double m = std::sqrt(a * a + b * b + c * c);

    residuals[0] = m / k;

    double dm_dx = (b * (lpl_.z() - lpj_.z()) + c * (lpj_.y() - lpl_.y())) / m;
    double dm_dy = (a * (lpj_.z() - lpl_.z()) - c * (lpj_.x() - lpl_.x())) / m;
    double dm_dz = (-a * (lpj_.y() - lpl_.y()) + b * (lpj_.x() - lpl_.x())) / m;

    double sr = std::sin(parameters[0][3]);
    double cr = std::cos(parameters[0][3]);
    double sp = std::sin(parameters[0][4]);
    double cp = std::cos(parameters[0][4]);
    double sy = std::sin(parameters[0][5]);
    double cy = std::cos(parameters[0][5]);

    double dx_dr = (cy * sp * cr + sr * sy) * cp_.y() + (sy * cr - cy * sr * sp) * cp_.z();
    double dy_dr = (-cy * sr + sy * sp * cr) * cp_.y() + (-sr * sy * sp - cy * cr) * cp_.z();
    double dz_dr = cp * cr * cp_.y() - cp * sr * cp_.z();

    double dx_dp = -cy * sp * cp_.x() + cy * cp * sr * cp_.y() + cy * cr * cp * cp_.z();
    double dy_dp = -sp * sy * cp_.x() + sy * cp * sr * cp_.y() + cr * sr * cp * cp_.z();
    double dz_dp = -cp * cp_.x() - sp * sr * cp_.y() - sp * cr * cp_.z();

    double dx_dy = -sy * cp * cp_.x() - (sy * sp * sr + cr * cy) * cp_.y() + (cy * sr - sy * cr * sp) * cp_.z();
    double dy_dy = cp * cy * cp_.x() + (-sy * cr + cy * sp * sr) * cp_.y() + (cy * cr * sp + sy * sr) * cp_.z();
    double dz_dy = 0.;

    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = dm_dx / k;
      jacobians[0][1] = dm_dy / k;
      jacobians[0][2] = 0.;
      jacobians[0][3] = 0.;
      jacobians[0][4] = 0.;
      jacobians[0][5] = (dm_dx * dx_dy + dm_dy * dy_dy + dm_dz * dz_dy) / k;
      // printf("lp: %.3f, %.3f, %.3f; lpj: %.3f, %.3f, %.3f; lpl: %.3f, %.3f, %.3f", lp.x(), lp.y(), lp.z(), lpj_.x(), lpj_.y(), lpj_.z(), lpl_.x(), lpl_.y(), lpl_.z());
      // printf("residual: %.3f\n", residuals[0]);
      // printf("J: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", jacobians[0][0], jacobians[0][1], jacobians[0][2], jacobians[0][3], jacobians[0][4], jacobians[0][5]);
    }

    return true;
  }

private:
  Eigen::Vector3d cp_;        // under t frame
  Eigen::Vector3d lpj_, lpl_; // under t-1 frame
};

class SurfCostFunction : public ceres::SizedCostFunction<1, 6>
{
public:
  SurfCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl, Eigen::Vector3d lpm) : cp_(cp), lpj_(lpj), lpl_(lpl), lpm_(lpm) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
  {
    Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameters[0][4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameters[0][3], Eigen::Vector3d::UnitX())) * cp_ + Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);
    double a = (lpj_.y() - lpl_.y()) * (lpj_.z() - lpm_.z()) - (lpj_.z() - lpl_.z()) * (lpj_.y() - lpm_.y());
    double b = (lpj_.z() - lpl_.z()) * (lpj_.x() - lpm_.x()) - (lpj_.x() - lpl_.x()) * (lpj_.z() - lpm_.z());
    double c = (lpj_.x() - lpl_.x()) * (lpj_.y() - lpm_.y()) - (lpj_.y() - lpl_.y()) * (lpj_.x() - lpm_.x());
    a *= a;
    b *= b;
    c *= c;
    double m = std::sqrt(std::pow((lp.x() - lpj_.x()), 2) * a + std::pow((lp.y() - lpj_.y()), 2) * b + std::pow((lp.z() - lpj_.z()), 2) * c);
    double k = std::sqrt(a + b + c);

    residuals[0] = m / k;

    double tmp = m * k;

    double dm_dx = ((lp.x() - lpj_.x()) * a) / tmp;
    double dm_dy = ((lp.y() - lpj_.y()) * b) / tmp;
    double dm_dz = ((lp.z() - lpj_.z()) * c) / tmp;

    double sr = std::sin(parameters[0][3]);
    double cr = std::cos(parameters[0][3]);
    double sp = std::sin(parameters[0][4]);
    double cp = std::cos(parameters[0][4]);
    double sy = std::sin(parameters[0][5]);
    double cy = std::cos(parameters[0][5]);

    double dx_dr = (cy * sp * cr + sr * sy) * cp_.y() + (sy * cr - cy * sr * sp) * cp_.z();
    double dy_dr = (-cy * sr + sy * sp * cr) * cp_.y() + (-sr * sy * sp - cy * cr) * cp_.z();
    double dz_dr = cp * cr * cp_.y() - cp * sr * cp_.z();

    double dx_dp = -cy * sp * cp_.x() + cy * cp * sr * cp_.y() + cy * cr * cp * cp_.z();
    double dy_dp = -sp * sy * cp_.x() + sy * cp * sr * cp_.y() + cr * sr * cp * cp_.z();
    double dz_dp = -cp * cp_.x() - sp * sr * cp_.y() - sp * cr * cp_.z();

    double dx_dy = -sy * cp * cp_.x() - (sy * sp * sr + cr * cy) * cp_.y() + (cy * sr - sy * cr * sp) * cp_.z();
    double dy_dy = cp * cy * cp_.x() + (-sy * cr + cy * sp * sr) * cp_.y() + (cy * cr * sp + sy * sr) * cp_.z();
    double dz_dy = 0.;

    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = 0.;
      jacobians[0][1] = 0.;
      jacobians[0][2] = dm_dz / k;
      jacobians[0][3] = 0.;
      jacobians[0][4] = 0.;
      jacobians[0][5] = 0.;
    }

    return true;
  }

private:
  Eigen::Vector3d cp_;
  Eigen::Vector3d lpj_, lpl_, lpm_;
};

class LidarEdgeCostFunction : public ceres::SizedCostFunction<1, 6>
{
public:
  LidarEdgeCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl) : cp_(cp), lpj_(lpj), lpl_(lpl) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
  {
    Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameters[0][4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameters[0][3], Eigen::Vector3d::UnitX())) * cp_ + Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);
    double k = std::sqrt(std::pow(lpj_.x() - lpl_.x(), 2) + std::pow(lpj_.y() - lpl_.y(), 2) + std::pow(lpj_.z() - lpl_.z(), 2));
    double a = (lp.y() - lpj_.y()) * (lp.z() - lpl_.z()) - (lp.z() - lpj_.z()) * (lp.y() - lpl_.y());
    double b = (lp.z() - lpj_.z()) * (lp.x() - lpl_.x()) - (lp.x() - lpj_.x()) * (lp.z() - lpl_.z());
    double c = (lp.x() - lpj_.x()) * (lp.y() - lpl_.y()) - (lp.y() - lpj_.y()) * (lp.x() - lpl_.x());
    double m = std::sqrt(a * a + b * b + c * c);

    residuals[0] = m / k;

    double dm_dx = (b * (lpl_.z() - lpj_.z()) + c * (lpj_.y() - lpl_.y())) / m;
    double dm_dy = (a * (lpj_.z() - lpl_.z()) - c * (lpj_.x() - lpl_.x())) / m;
    double dm_dz = (-a * (lpj_.y() - lpl_.y()) + b * (lpj_.x() - lpl_.x())) / m;

    double sr = std::sin(parameters[0][3]);
    double cr = std::cos(parameters[0][3]);
    double sp = std::sin(parameters[0][4]);
    double cp = std::cos(parameters[0][4]);
    double sy = std::sin(parameters[0][5]);
    double cy = std::cos(parameters[0][5]);

    double dx_dr = (cy * sp * cr + sr * sy) * cp_.y() + (sy * cr - cy * sr * sp) * cp_.z();
    double dy_dr = (-cy * sr + sy * sp * cr) * cp_.y() + (-sr * sy * sp - cy * cr) * cp_.z();
    double dz_dr = cp * cr * cp_.y() - cp * sr * cp_.z();

    double dx_dp = -cy * sp * cp_.x() + cy * cp * sr * cp_.y() + cy * cr * cp * cp_.z();
    double dy_dp = -sp * sy * cp_.x() + sy * cp * sr * cp_.y() + cr * sr * cp * cp_.z();
    double dz_dp = -cp * cp_.x() - sp * sr * cp_.y() - sp * cr * cp_.z();

    double dx_dy = -sy * cp * cp_.x() - (sy * sp * sr + cr * cy) * cp_.y() + (cy * sr - sy * cr * sp) * cp_.z();
    double dy_dy = cp * cy * cp_.x() + (-sy * cr + cy * sp * sr) * cp_.y() + (cy * cr * sp + sy * sr) * cp_.z();
    double dz_dy = 0.;

    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = dm_dx / k;
      jacobians[0][1] = dm_dy / k;
      jacobians[0][2] = dm_dz / k;
      jacobians[0][3] = (dm_dx * dx_dr + dm_dy * dy_dr + dm_dz * dz_dr) / k;
      jacobians[0][4] = (dm_dx * dx_dp + dm_dy * dy_dp + dm_dz * dz_dp) / k;
      jacobians[0][5] = (dm_dx * dx_dy + dm_dy * dy_dy + dm_dz * dz_dy) / k;
      // printf("lp: %.3f, %.3f, %.3f; lpj: %.3f, %.3f, %.3f; lpl: %.3f, %.3f, %.3f", lp.x(), lp.y(), lp.z(), lpj_.x(), lpj_.y(), lpj_.z(), lpl_.x(), lpl_.y(), lpl_.z());
      // printf("residual: %.3f\n", residuals[0]);
      // printf("J: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", jacobians[0][0], jacobians[0][1], jacobians[0][2], jacobians[0][3], jacobians[0][4], jacobians[0][5]);
    }

    return true;
  }

private:
  Eigen::Vector3d cp_;        // under t frame
  Eigen::Vector3d lpj_, lpl_; // under t-1 frame
};

class LidarPlaneCostFunction : public ceres::SizedCostFunction<1, 6>
{
public:
  LidarPlaneCostFunction(Eigen::Vector3d cp, Eigen::Vector3d plane_unit_norm,
                         double negative_OA_dot_norm) : cp_(cp), plane_unit_norm_(plane_unit_norm),
                                                        negative_OA_dot_norm_(negative_OA_dot_norm) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
  {
    Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameters[0][4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(parameters[0][3], Eigen::Vector3d::UnitX())) * cp_ + Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);
    residuals[0] = plane_unit_norm_.dot(lp) + negative_OA_dot_norm_;
    Eigen::Vector3d df_dxyz = plane_unit_norm_;

    double sr = std::sin(parameters[0][3]);
    double cr = std::cos(parameters[0][3]);
    double sp = std::sin(parameters[0][4]);
    double cp = std::cos(parameters[0][4]);
    double sy = std::sin(parameters[0][5]);
    double cy = std::cos(parameters[0][5]);

    double dx_dr = (cy * sp * cr + sr * sy) * cp_.y() + (sy * cr - cy * sr * sp) * cp_.z();
    double dy_dr = (-cy * sr + sy * sp * cr) * cp_.y() + (-sr * sy * sp - cy * cr) * cp_.z();
    double dz_dr = cp * cr * cp_.y() - cp * sr * cp_.z();

    double dx_dp = -cy * sp * cp_.x() + cy * cp * sr * cp_.y() + cy * cr * cp * cp_.z();
    double dy_dp = -sp * sy * cp_.x() + sy * cp * sr * cp_.y() + cr * sr * cp * cp_.z();
    double dz_dp = -cp * cp_.x() - sp * sr * cp_.y() - sp * cr * cp_.z();

    double dx_dy = -sy * cp * cp_.x() - (sy * sp * sr + cr * cy) * cp_.y() + (cy * sr - sy * cr * sp) * cp_.z();
    double dy_dy = cp * cy * cp_.x() + (-sy * cr + cy * sp * sr) * cp_.y() + (cy * cr * sp + sy * sr) * cp_.z();
    double dz_dy = 0.;

    if (jacobians && jacobians[0])
    {
      jacobians[0][0] = df_dxyz.x();
      jacobians[0][1] = df_dxyz.y();
      jacobians[0][2] = df_dxyz.z();
      jacobians[0][3] = df_dxyz.x() * dx_dr + df_dxyz.y() * dy_dr + df_dxyz.z() * dz_dr;
      jacobians[0][4] = df_dxyz.x() * dx_dp + df_dxyz.y() * dy_dp + df_dxyz.z() * dz_dp;
      jacobians[0][5] = df_dxyz.x() * dx_dy + df_dxyz.y() * dy_dy + df_dxyz.z() * dz_dy;
    }

    return true;
  }

private:
  Eigen::Vector3d cp_;
  Eigen::Vector3d plane_unit_norm_;
  double negative_OA_dot_norm_;
};

#endif
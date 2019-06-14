#ifndef _LOAM_UTILITY_
#define _LOAM_UTILITY_

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <array>
#include <thread>
#include <chrono>
#include <mutex>
#include <memory>
#include <limits>

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

#define RAD2ANGLE(x) ((x) * 180.0 / M_PI)
#define ANGLE2RAD(x) ((x) / 180.0 * M_PI)

const int N_SCAN = 16;
const float ang_res_x = 0.18; // 10Hz
const float ang_res_y = 2.0;
const float scan_period = 0.1; // 10Hz

const int Horizon_SCAN = 360.0 / ang_res_x + 0.5;
const float ang_bottom = 15.0;
const int ground_scan_id = 7;
const float sensor_mount_ang = 0.;  // 向下为正，向上为负

const float seg_alpha_x = ANGLE2RAD(ang_res_x);
const float seg_alpha_y = ANGLE2RAD(ang_res_y);
// 可以调调参数
const float seg_theta = 1.0472;
const int seg_valid_point_num = 5;
const int seg_valid_line_num = 3;

// (use_imu && use_odom) == false
const bool use_imu = true;
const bool use_odom = false;
const int imu_queue_length = 1000;
const int odom_queue_length = 1000;

const float nearest_feature_dist = 25.; // sqaured, 对应 5m

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

#endif
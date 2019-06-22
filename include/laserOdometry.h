#include "utility.h"

namespace loam
{
using namespace std;
class LaserOdometry : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_segmented_cloud_;
  ros::Subscriber sub_segmented_info_;
  ros::Subscriber sub_outlier_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_surf_;
  ros::Publisher pub_surf_less_;
  ros::Publisher pub_corner_;
  ros::Publisher pub_corner_less_;
  ros::Publisher pub_undistorted_pc_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_surf_last_;
  ros::Publisher pub_corner_last_;
  ros::Publisher pub_outlier_last_;
  tf::TransformBroadcaster tf_broad_;

  queue<sensor_msgs::PointCloud2ConstPtr> seg_cloud_buf_;
  queue<alego::cloud_infoConstPtr> seg_info_buf_;
  queue<sensor_msgs::PointCloud2ConstPtr> outlier_buf_;
  queue<nav_msgs::OdometryConstPtr> odom_buf_;
  queue<sensor_msgs::ImuConstPtr> imu_buf_;

  int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  std::array<double, imu_queue_length> imu_time_;
  std::array<double, imu_queue_length> imu_roll_;
  std::array<double, imu_queue_length> imu_pitch_;
  std::array<double, imu_queue_length> imu_yaw_;
  std::array<double, imu_queue_length> imu_shift_x_;
  std::array<double, imu_queue_length> imu_shift_y_;
  std::array<double, imu_queue_length> imu_shift_z_;
  std::array<double, imu_queue_length> imu_velo_x_;
  std::array<double, imu_queue_length> imu_velo_y_;
  std::array<double, imu_queue_length> imu_velo_z_;

  // 里程计相关
  int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;
  std::array<nav_msgs::OdometryConstPtr, odom_queue_length> odom_queue_;
  std::array<double, odom_queue_length> odom_roll_;
  std::array<double, odom_queue_length> odom_pitch_;
  std::array<double, odom_queue_length> odom_yaw_;

  std::array<double, N_SCAN * Horizon_SCAN> cloud_curvature_;
  std::array<bool, N_SCAN * Horizon_SCAN> cloud_neighbor_picked_;
  std::array<int, N_SCAN * Horizon_SCAN> cloud_label_;
  std::array<int, N_SCAN * Horizon_SCAN> cloud_sort_idx_;

  bool system_initialized_;

  PointCloudT::Ptr surf_last_;
  PointCloudT::Ptr corner_last_;
  PointCloudT::Ptr outlier_last_;

  pcl::KdTreeFLANN<PointT>::Ptr kd_surf_last_;
  pcl::KdTreeFLANN<PointT>::Ptr kd_corner_last_;

  double params_[6];        // 0-2: t, 3-5: rpy
  double params_surf_[3];   // z, roll, pitch
  double params_corner_[3]; // x, y, yaw
  Eigen::Vector3d t_w_cur_;
  Eigen::Matrix3d r_w_cur_;

  std::mutex m_buf_;

  void mainLoop();

  void adjustDistortion(PointCloudT::Ptr cloud, double scan_time);
  void transformToStart(const PointT &pi, PointT &po);

  void segCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  void segInfoHandler(const alego::cloud_infoConstPtr &msg);
  void outlierHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

  void imuHandler(const sensor_msgs::ImuConstPtr &msg);
  void odomHandler(const nav_msgs::OdometryConstPtr &msg);
};
} // namespace loam

PLUGINLIB_EXPORT_CLASS(loam::LaserOdometry, nodelet::Nodelet)
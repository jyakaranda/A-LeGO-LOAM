#include "utility.h"
#include <std_srvs/Empty.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace loam
{
using namespace std;
using namespace gtsam;
class LaserMapping : public nodelet::Nodelet
{
public:
  LaserMapping() {}
  ~LaserMapping() {}
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_surf_last_;
  ros::Subscriber sub_corner_last_;
  ros::Subscriber sub_outlier_last_;
  ros::Subscriber sub_laser_odom_;

  ros::Publisher pub_cloud_surround_;
  ros::Publisher pub_odom_aft_mapped_;
  ros::Publisher pub_keyposes_;

  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;
  ros::Publisher pub_recent_keyframes_;

  ros::ServiceServer srv_save_map_;

  tf::TransformBroadcaster tf_broadcaster_;

  NonlinearFactorGraph gtSAMgraph_;
  Values init_estimate_;
  Values opt_estimate_;
  ISAM2 *isam_;
  Values isam_cur_estimate_;
  noiseModel::Diagonal::shared_ptr prior_noise_;
  noiseModel::Diagonal::shared_ptr odom_noise_;
  noiseModel::Diagonal::shared_ptr constraint_noise_;

  // 回环检测相关
  bool loop_closed_;
  int latest_history_frame_id_;
  int closest_history_frame_id_;
  PointCloudT::Ptr latest_keyframe_;
  PointCloudT::Ptr near_history_keyframes_;

  std::mutex mtx_;
  std::thread loop_thread_;
  std::thread main_thread_;
  std::thread visualize_thread_;

  double history_search_radius_; // 回环检测参数
  int history_search_num_;
  double history_fitness_score_;
  bool loop_closure_enabled_;

  PointCloudT::Ptr surf_last_;
  PointCloudT::Ptr corner_last_;
  PointCloudT::Ptr outlier_last_;

  vector<PointCloudT::Ptr> corner_frames_;
  vector<PointCloudT::Ptr> surf_frames_;
  vector<PointCloudT::Ptr> outlier_frames_;

  PointCloudT::Ptr cloud_keyposes_3d_;
  pcl::PointCloud<PointTypePose>::Ptr cloud_keyposes_6d_;

  PointCloudT::Ptr corner_from_map_;
  PointCloudT::Ptr surf_from_map_;
  // PointCloudT::Ptr outlier_from_map_;
  PointCloudT::Ptr corner_from_map_ds_;
  PointCloudT::Ptr surf_from_map_ds_;

  PointCloudT::Ptr laser_corner_;
  PointCloudT::Ptr laser_surf_;
  PointCloudT::Ptr laser_outlier_;
  PointCloudT::Ptr laser_surf_total_;
  PointCloudT::Ptr laser_corner_ds_;
  PointCloudT::Ptr laser_surf_ds_;
  PointCloudT::Ptr laser_outlier_ds_;
  PointCloudT::Ptr laser_surf_total_ds_;
  double time_laser_corner_;
  double time_laser_surf_;
  double time_laser_outlier_;
  double time_laser_odom_;
  bool new_laser_corner_;
  bool new_laser_surf_;
  bool new_laser_outlier_;
  bool new_laser_odom_;

  pcl::KdTreeFLANN<PointT>::Ptr kd_corner_map_;
  pcl::KdTreeFLANN<PointT>::Ptr kd_surf_map_;
  pcl::KdTreeFLANN<PointT>::Ptr kd_keyposes_;
  vector<int> point_idx_;
  vector<float> point_dist_;

  pcl::VoxelGrid<PointT> ds_corner_;
  pcl::VoxelGrid<PointT> ds_surf_;
  pcl::VoxelGrid<PointT> ds_outlier_;
  pcl::VoxelGrid<PointT> ds_keyposes_;
  pcl::VoxelGrid<PointT> ds_history_keyframes_;

  double min_keyframe_dist_;

  // 回环检测使用
  deque<PointCloudT::Ptr> recent_corner_keyframes_;
  deque<PointCloudT::Ptr> recent_surf_keyframes_;
  deque<PointCloudT::Ptr> recent_outlier_keyframes_;
  int recent_keyframe_search_num_;
  int latest_frame_id_;
  Eigen::Matrix4d correction_;

  // 无回环检测使用
  PointCloudT::Ptr surround_keyposes_;
  PointCloudT::Ptr surround_keyposes_ds_;
  vector<int> surround_exist_keypose_id_;
  vector<PointCloudT::Ptr> surround_corner_keyframes_;
  vector<PointCloudT::Ptr> surround_surf_keyframes_;
  vector<PointCloudT::Ptr> surround_outlier_keyframes_;
  double surround_keyframe_search_radius_;

  double params_[6];
  Eigen::Vector3d t_map2odom_;
  Eigen::Quaterniond q_map2odom_;
  Eigen::Vector3d t_odom2laser_;
  Eigen::Quaterniond q_odom2laser_;
  Eigen::Vector3d t_map2laser_;
  Eigen::Quaterniond q_map2laser_;

  void mainLoop();

  void surfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  void cornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  void outlierLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  void laserOdomHandler(const nav_msgs::OdometryConstPtr &msg);

  void transformAssociateToMap();
  void extractSurroundingKeyFrames();
  void downsampleCurrentScan();
  void scan2MapOptimization();
  void transformUpdate();
  bool saveKeyFramesAndFactor();
  void correctPoses();
  void publish();

  void visualizeGlobalMapThread();

  void loopClosureThread();
  bool detectLoopClosure();
  void performLoopClosure();
  PointCloudT::Ptr transformPointCloud(const PointCloudT::ConstPtr cloud_in, const PointTypePose &trans)
  {
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
                                                .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    PointCloudT::Ptr tf_cloud(new PointCloudT());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
  }
  PointCloudT::Ptr transformPointCloud(const PointCloudT::ConstPtr cloud_in, const PointTypePose &trans, int idx)
  {
    PointCloudT::Ptr tf_cloud = transformPointCloud(cloud_in, trans);
    for (auto &p : tf_cloud->points)
    {
      p.intensity = idx;
    }
    return tf_cloud;
  }
  void pointAssociateToMap(const PointT &p_in, PointT &p_out)
  {
    Eigen::Vector3d out = q_map2laser_ * Eigen::Vector3d(p_in.x, p_in.y, p_in.z) + t_map2laser_;
    p_out.x = out.x();
    p_out.y = out.y();
    p_out.z = out.z();
    p_out.intensity = p_in.intensity;
  }
  bool saveMapCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};
} // namespace loam

PLUGINLIB_EXPORT_CLASS(loam::LaserMapping, nodelet::Nodelet)
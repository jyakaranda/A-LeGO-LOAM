#include "alego/utility.h"

namespace loam
{
using namespace std;

class ImageProjection : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pc_;
  ros::Publisher pub_segmented_cloud_;
  ros::Publisher pub_seg_info_;
  ros::Publisher pub_outlier_;
  sensor_msgs::PointCloud2Ptr segmented_cloud_msg_;
  alego::cloud_infoPtr seg_info_msg_;
  sensor_msgs::PointCloud2Ptr outlier_cloud_msg_;

  PointCloudT::Ptr full_cloud_;
  PointCloudT::Ptr segmented_cloud_;
  PointCloudT::Ptr outlier_cloud_;

  Eigen::Matrix<double, Eigen::Dynamic, Horizon_SCAN> range_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Horizon_SCAN> label_mat_;
  Eigen::Matrix<bool, Eigen::Dynamic, Horizon_SCAN> ground_mat_;

  int label_cnt_;
  vector<pair<int, int>> neighbor_iter_;

  // params

public:
  ImageProjection()
  {
  }
  virtual void onInit();

private:
  void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  void labelComponents(int row, int col);

  void publish();

  template <typename PointT>
  void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres);
};
} // namespace loam

PLUGINLIB_EXPORT_CLASS(loam::ImageProjection, nodelet::Nodelet);
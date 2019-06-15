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
  ros::Publisher pub_path_;

  queue<sensor_msgs::PointCloud2ConstPtr> seg_cloud_buf_;
  queue<alego::cloud_infoConstPtr> seg_info_buf_;
  queue<sensor_msgs::PointCloud2ConstPtr> outlier_buf_;
  queue<nav_msgs::OdometryConstPtr> odom_buf_;
  queue<sensor_msgs::ImuConstPtr> imu_buf_;

  int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  std::array<double, imu_queue_length> imu_time_;
  std::array<float, imu_queue_length> imu_roll_;
  std::array<float, imu_queue_length> imu_pitch_;
  std::array<float, imu_queue_length> imu_yaw_;
  std::array<float, imu_queue_length> imu_shift_x_;
  std::array<float, imu_queue_length> imu_shift_y_;
  std::array<float, imu_queue_length> imu_shift_z_;
  std::array<float, imu_queue_length> imu_velo_x_;
  std::array<float, imu_queue_length> imu_velo_y_;
  std::array<float, imu_queue_length> imu_velo_z_;

  // 里程计相关
  int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;
  std::array<nav_msgs::OdometryConstPtr, odom_queue_length> odom_queue_;
  std::array<float, odom_queue_length> odom_roll_;
  std::array<float, odom_queue_length> odom_pitch_;
  std::array<float, odom_queue_length> odom_yaw_;

  std::array<float, N_SCAN * Horizon_SCAN> cloud_curvature_;
  std::array<bool, N_SCAN * Horizon_SCAN> cloud_neighbor_picked_;
  std::array<int, N_SCAN * Horizon_SCAN> cloud_label_;
  std::array<int, N_SCAN * Horizon_SCAN> cloud_sort_idx_;

  bool system_initialized_;

  PointCloudT::Ptr surf_last_;
  PointCloudT::Ptr corner_last_;

  pcl::KdTreeFLANN<PointT>::Ptr kd_surf_last_;
  pcl::KdTreeFLANN<PointT>::Ptr kd_corner_last_;

  double params_[6]; // 0-2: t, 3-5: rpy
  Eigen::Vector3f t_w_cur_;
  Eigen::Matrix3f r_w_cur_;

  std::mutex m_buf_;

  void mainLoop();

  void adjustDistortion(PointCloudT::Ptr cloud, double scan_time);
  void transformToStart(const PointT &pi, PointT &po);

  void segCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  void segInfoHandler(const alego::cloud_infoConstPtr &msg);
  void outlierHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

  void imuHandler(const sensor_msgs::ImuConstPtr &msg);
  void odomHandler(const nav_msgs::OdometryConstPtr &msg);

  class CornerCostFunction : public ceres::SizedCostFunction<1, 6>
  {
  public:
    CornerCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl) : cp_(cp), lpj_(lpj), lpl_(lpl) {}
    ~CornerCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

  private:
    Eigen::Vector3d cp_;        // under t frame
    Eigen::Vector3d lpj_, lpl_; // under t-1 frame
  };

  class SurfCostFunction : public ceres::SizedCostFunction<1, 6>
  {
  public:
    SurfCostFunction(Eigen::Vector3d cp, Eigen::Vector3d lpj, Eigen::Vector3d lpl, Eigen::Vector3d lpm) : cp_(cp), lpj_(lpj), lpl_(lpl), lpm_(lpm) {}
    ~SurfCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

  private:
    Eigen::Vector3d cp_;
    Eigen::Vector3d lpj_, lpl_, lpm_;
  };

  struct LidarEdgeFactor
  {
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_, double s_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

    template <typename T>
    bool operator()(const T *t, T *residual) const
    {

      Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
      Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
      Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

      //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
      Eigen::Quaternion<T> q_last_curr = Eigen::AngleAxis<T>(t[5], Eigen::Matrix<T, 3, 1>::UnitZ()) * Eigen::AngleAxis<T>(t[4], Eigen::Matrix<T, 3, 1>::UnitY()) * Eigen::AngleAxis<T>(t[3], Eigen::Matrix<T, 3, 1>::UnitX());
      Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
      q_last_curr = q_identity.slerp(T(s), q_last_curr);
      Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

      Eigen::Matrix<T, 3, 1> lp;
      lp = q_last_curr * cp + t_last_curr;

      Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
      Eigen::Matrix<T, 3, 1> de = lpa - lpb;

      residual[0] = nu.norm() / de.norm();
      // residual[0] = nu.x() / de.norm();
      // residual[1] = nu.y() / de.norm();
      // residual[2] = nu.z() / de.norm();

      return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const double s_)
    {
      return (new ceres::AutoDiffCostFunction<
              LidarEdgeFactor, 1, 6>(
          new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
  };

  struct LidarPlaneFactor
  {
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
          last_point_m(last_point_m_), s(s_)
    {
      ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
      ljm_norm.normalize();
    }

    template <typename T>
    bool operator()(const T *t, T *residual) const
    {

      Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
      Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
      //Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
      //Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
      Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

      //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
      // Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
      Eigen::Quaternion<T> q_last_curr = Eigen::AngleAxis<T>(t[5], Eigen::Matrix<T, 3, 1>::UnitZ()) * Eigen::AngleAxis<T>(t[4], Eigen::Matrix<T, 3, 1>::UnitY()) * Eigen::AngleAxis<T>(t[3], Eigen::Matrix<T, 3, 1>::UnitX());
      Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
      q_last_curr = q_identity.slerp(T(s), q_last_curr);
      Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

      Eigen::Matrix<T, 3, 1> lp;
      lp = q_last_curr * cp + t_last_curr;

      residual[0] = (lp - lpj).dot(ljm);

      return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
                                       const double s_)
    {
      return (new ceres::AutoDiffCostFunction<
              LidarPlaneFactor, 1, 6>(
          new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;
  };
};
} // namespace loam

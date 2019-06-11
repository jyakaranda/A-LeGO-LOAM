#include "utility.h"
#include "laserOdometry.h"

PLUGINLIB_EXPORT_CLASS(loam::LaserOdometry, nodelet::Nodelet)

namespace loam
{

void LaserOdometry::onInit()
{
  cout << "--------- LaserOdometry init --------------" << endl;
  TicToc t_init;
 
  if (use_imu && use_odom != false)
  {
    ROS_ERROR("set use_imu=false when use_odom=true");
    return;
  }

  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;
  imu_time_.fill(0);
  imu_roll_.fill(0);
  imu_pitch_.fill(0);
  imu_yaw_.fill(0);
  imu_velo_x_.fill(0);
  imu_velo_y_.fill(0);
  imu_velo_z_.fill(0);
  imu_shift_x_.fill(0);
  imu_shift_y_.fill(0);
  imu_shift_z_.fill(0);

  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  pub_corner_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner", 10);
  pub_corner_less_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner_less", 10);
  pub_surf_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf", 10);
  pub_surf_less_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf_less", 10);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted", 10);

  sub_segmented_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 10, boost::bind(&LaserOdometry::segCloudHandler, this, _1));
  sub_segmented_info_ = nh_.subscribe<alego::cloud_info>("/seg_info", 10, boost::bind(&LaserOdometry::segInfoHandler, this, _1));
  sub_outlier_ = nh_.subscribe<sensor_msgs::PointCloud2>("/outlier", 10, boost::bind(&LaserOdometry::outlierHandler, this, _1));
  if (use_imu)
  {
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 100, boost::bind(&LaserOdometry::imuHandler, this, _1));
  }
  else if (use_odom)
  {
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 100, boost::bind(&LaserOdometry::odomHandler, this, _1));
  }

  cout << "LaserOdometry onInit end: " << t_init.toc() << endl;

  static std::thread main_thread(&LaserOdometry::mainLoop, this);
}

void LaserOdometry::mainLoop()
{
  ros::Rate rate(100);
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    if (!seg_cloud_buf_.empty() && !seg_info_buf_.empty() && !outlier_buf_.empty())
    {
      double t1 = seg_cloud_buf_.front()->header.stamp.toSec();
      double t2 = seg_info_buf_.front()->header.stamp.toSec();
      double t3 = outlier_buf_.front()->header.stamp.toSec();
      if (abs(t1 - t2) > 0.2 || abs(t1 - t3) > 0.2)
      {
        NODELET_WARN("unsync msg");
        while (!seg_cloud_buf_.empty())
        {
          seg_cloud_buf_.pop();
        }
        while (!seg_info_buf_.empty())
        {
          seg_info_buf_.pop();
        }
        while (!outlier_buf_.empty())
        {
          outlier_buf_.pop();
        }
        continue;
      }

      PointCloudT::Ptr seg_cloud(new PointCloudT());
      pcl::fromROSMsg(*seg_cloud_buf_.front(), *seg_cloud);
      adjustDistortion(seg_cloud, t1);
      seg_cloud_buf_.pop();
      seg_info_buf_.pop();
      outlier_buf_.pop();
    }
  }
}

void LaserOdometry::adjustDistortion(PointCloudT::Ptr cloud, double scan_time)
{
  NODELET_INFO("scan_time %.6f", scan_time);
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  float start_ori = seg_info_buf_.front()->startOrientation;
  float end_ori = seg_info_buf_.front()->endOrientation;

  float ori_diff = seg_info_buf_.front()->orientationDiff;
  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;

  for (int i = 0; i < cloud_size; ++i)
  {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed)
    {
      if (ori_h < start_ori - M_PI * 0.5)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > start_ori + M_PI * 1.5)
      {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI)
      {
        half_passed = true;
      }
    }
    else
    {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > end_ori + 0.5 * M_PI)
      {
        ori_h -= 2 * M_PI;
      }
    }

    double rel_time = (ori_h - start_ori) / ori_diff * scan_period;
    double cur_time = scan_time + rel_time;

    if (use_imu)
    {
      if (imu_ptr_last_ > 0)
      {
        imu_ptr_front_ = imu_ptr_last_iter_;
        while (imu_ptr_front_ != imu_ptr_last_)
        {
          if (cur_time < imu_time_[imu_ptr_front_])
          {
            break;
          }
          imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_length;
        }
        if (abs(cur_time - imu_time_[imu_ptr_front_]) > scan_period)
        {
          NODELET_WARN_COND(i < 5 ,"unsync imu and pc msg %d, %d, %.6f, %.6f", imu_ptr_front_, imu_ptr_last_, cur_time, imu_time_[imu_ptr_front_]);
          continue;
        }

        if (cur_time > imu_time_[imu_ptr_front_])
        {
          rpy_cur(0) = imu_roll_[imu_ptr_front_];
          rpy_cur(1) = imu_pitch_[imu_ptr_front_];
          rpy_cur(2) = imu_yaw_[imu_ptr_front_];
          shift_cur(0) = imu_shift_x_[imu_ptr_front_];
          shift_cur(1) = imu_shift_y_[imu_ptr_front_];
          shift_cur(2) = imu_shift_z_[imu_ptr_front_];
          velo_cur(0) = imu_velo_x_[imu_ptr_front_];
          velo_cur(1) = imu_velo_y_[imu_ptr_front_];
          velo_cur(2) = imu_velo_z_[imu_ptr_front_];
        }
        else
        {
          int imu_ptr_back = (imu_ptr_front_ - 1 + imu_queue_length) % imu_queue_length;
          float ratio_front = (cur_time - imu_time_[imu_ptr_back]) / (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
          float ratio_back = 1. - ratio_front;
          rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
          rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
          rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
          shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
          shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
          shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
          velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
          velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
          velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
        }

        r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

        if (i == 0)
        {
          rpy_start = rpy_cur;
          shift_start = shift_cur;
          velo_start = velo_cur;
          r_s_i = r_c.inverse();
        }
        else
        {
          shift_from_start = shift_cur - shift_start - velo_start * rel_time;
          adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
          p.x = adjusted_p.x();
          p.y = adjusted_p.y();
          p.z = adjusted_p.z();
        }
        imu_ptr_last_iter_ = imu_ptr_front_;
      }
    }
    else if (use_odom)
    {
      if (odom_ptr_last_ > 0)
      {
        odom_ptr_front_ = odom_ptr_last_iter_;
        while (odom_ptr_front_ != odom_ptr_last_)
        {
          if (cur_time > odom_queue_[odom_ptr_front_]->header.stamp.toSec())
          {
            break;
          }
          odom_ptr_front_ = (odom_ptr_front_ + 1) % odom_queue_length;
        }
        if (cur_time > odom_queue_[odom_ptr_front_]->header.stamp.toSec())
        {
          rpy_cur(0) = odom_roll_[odom_ptr_front_];
          rpy_cur(1) = odom_pitch_[odom_ptr_front_];
          rpy_cur(2) = odom_yaw_[odom_ptr_front_];
          shift_cur(0) = odom_queue_[odom_ptr_front_]->pose.pose.position.x;
          shift_cur(1) = odom_queue_[odom_ptr_front_]->pose.pose.position.y;
          shift_cur(2) = odom_queue_[odom_ptr_front_]->pose.pose.position.z;
          velo_cur(0) = odom_queue_[odom_ptr_front_]->twist.twist.linear.x;
          velo_cur(1) = odom_queue_[odom_ptr_front_]->twist.twist.linear.y;
          velo_cur(2) = odom_queue_[odom_ptr_front_]->twist.twist.linear.z;
        }
        else
        {
          int odom_ptr_back = (odom_ptr_front_ - 1 + odom_queue_length) % odom_queue_length;
          float ratio_front = (cur_time - odom_queue_[odom_ptr_back]->header.stamp.toSec()) / (odom_queue_[odom_ptr_front_]->header.stamp.toSec() - odom_queue_[odom_ptr_back]->header.stamp.toSec());
          float ratio_back = 1. - ratio_front;
          rpy_cur(0) = ratio_front * odom_roll_[odom_ptr_front_] + ratio_back * odom_roll_[odom_ptr_back];
          rpy_cur(1) = ratio_front * odom_pitch_[odom_ptr_front_] + ratio_back * odom_pitch_[odom_ptr_back];
          rpy_cur(2) = ratio_front * odom_yaw_[odom_ptr_front_] + ratio_back * odom_yaw_[odom_ptr_back];
          shift_cur(0) = ratio_front * odom_queue_[odom_ptr_front_]->pose.pose.position.x + ratio_back * odom_queue_[odom_ptr_front_]->pose.pose.position.x;
          shift_cur(1) = ratio_front * odom_queue_[odom_ptr_front_]->pose.pose.position.y + ratio_back * odom_queue_[odom_ptr_front_]->pose.pose.position.y;
          shift_cur(2) = ratio_front * odom_queue_[odom_ptr_front_]->pose.pose.position.z + ratio_back * odom_queue_[odom_ptr_front_]->pose.pose.position.z;
          velo_cur(0) = ratio_front * odom_queue_[odom_ptr_front_]->twist.twist.linear.x + ratio_back * odom_queue_[odom_ptr_front_]->twist.twist.linear.x;
          velo_cur(1) = ratio_front * odom_queue_[odom_ptr_front_]->twist.twist.linear.y + ratio_back * odom_queue_[odom_ptr_front_]->twist.twist.linear.y;
          velo_cur(2) = ratio_front * odom_queue_[odom_ptr_front_]->twist.twist.linear.z + ratio_back * odom_queue_[odom_ptr_front_]->twist.twist.linear.z;
        }
        r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();
        if (i == 0)
        {
          rpy_start = rpy_cur;
          shift_start = shift_cur;
          velo_start = velo_cur;
          r_s_i = r_c.inverse();
        }
        else
        {
          shift_from_start = shift_cur - shift_start - velo_start * rel_time;
          adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
          p.x = adjusted_p.x();
          p.y = adjusted_p.y();
          p.z = adjusted_p.z();
        }
        odom_ptr_last_iter_ = odom_ptr_front_;
      }
    }
  }
  if (pub_undistorted_pc_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *msg);
    msg->header.stamp.fromSec(scan_time);
    msg->header.frame_id = "/laser";
    pub_undistorted_pc_.publish(msg);
  }
}

void LaserOdometry::segCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  m_buf_.lock();
  seg_cloud_buf_.push(msg);
  m_buf_.unlock();
}
void LaserOdometry::segInfoHandler(const alego::cloud_infoConstPtr &msg)
{
  m_buf_.lock();
  seg_info_buf_.push(msg);
  m_buf_.unlock();
}
void LaserOdometry::outlierHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  m_buf_.lock();
  outlier_buf_.push(msg);
  m_buf_.unlock();
}

void LaserOdometry::imuHandler(const sensor_msgs::ImuConstPtr &msg)
{
  imu_buf_.push(msg);

  double roll, pitch, yaw;
  tf::Quaternion ori;
  tf::quaternionMsgToTF(msg->orientation, ori);
  tf::Matrix3x3(ori).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_length;

  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_length) % imu_queue_length;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1.)
  {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;
  }
}
void LaserOdometry::odomHandler(const nav_msgs::OdometryConstPtr &msg)
{
  odom_buf_.push(msg);

  double roll, pitch, yaw;
  tf::Quaternion ori;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, ori);
  tf::Matrix3x3(ori).getRPY(roll, pitch, yaw);

  odom_ptr_last_ = (odom_ptr_last_ + 1) % odom_queue_length;
  odom_roll_[odom_ptr_last_] = roll;
  odom_pitch_[odom_ptr_last_] = pitch;
  odom_yaw_[odom_ptr_last_] = yaw;
}
} // namespace loam
#include "alego/laserOdometry.h"

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

  cloud_curvature_.fill(0);
  cloud_neighbor_picked_.fill(false);
  cloud_label_.fill(0);
  cloud_sort_idx_.fill(0);

  system_initialized_ = false;
  surf_last_.reset(new PointCloudT);
  corner_last_.reset(new PointCloudT);
  outlier_last_.reset(new PointCloudT);
  kd_surf_last_.reset(new pcl::KdTreeFLANN<PointT>);
  kd_corner_last_.reset(new pcl::KdTreeFLANN<PointT>);
  for (int i = 0; i < 6; ++i)
  {
    params_[i] = 0.;
  }
  t_w_cur_.setZero();
  r_w_cur_.setIdentity();

  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  pub_corner_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner", 10);
  pub_corner_less_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner_less", 10);
  pub_surf_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf", 10);
  pub_surf_less_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf_less", 10);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted", 10);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom/lidar", 10);
  pub_surf_last_ = nh_.advertise<sensor_msgs::PointCloud2>("/surf_last", 10);
  pub_corner_last_ = nh_.advertise<sensor_msgs::PointCloud2>("/corner_last", 10);
  pub_outlier_last_ = nh_.advertise<sensor_msgs::PointCloud2>("/outlier_last", 10);

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
        m_buf_.lock();
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
        m_buf_.unlock();
        continue;
      }

      // step1: adjustDistortion
      TicToc t_adj;
      PointCloudT::Ptr seg_cloud(new PointCloudT());
      pcl::fromROSMsg(*seg_cloud_buf_.front(), *seg_cloud);
      // adjustDistortion(seg_cloud, t1);
      NODELET_INFO("adjustDistortion %.3f ms", t_adj.toc());

      // step2: calculateSmoothness
      TicToc t_cal;
      int cloud_size = seg_cloud->points.size();
      alego::cloud_infoConstPtr seg_info = seg_info_buf_.front();
      for (int i = 5; i < cloud_size - 5; ++i)
      {
        double diff_range = seg_info->segmentedCloudRange[i - 5] + seg_info->segmentedCloudRange[i - 4] + seg_info->segmentedCloudRange[i - 3] + seg_info->segmentedCloudRange[i - 2] + seg_info->segmentedCloudRange[i - 1] - seg_info->segmentedCloudRange[i] * 10 + seg_info->segmentedCloudRange[i + 1] + seg_info->segmentedCloudRange[i + 2] + seg_info->segmentedCloudRange[i + 3] + seg_info->segmentedCloudRange[i + 4] + seg_info->segmentedCloudRange[i + 5];
        cloud_curvature_[i] = diff_range * diff_range;
        cloud_neighbor_picked_[i] = 0;
        cloud_label_[i] = 0;
        cloud_sort_idx_[i] = i;
      }
      // step3: markOccludedPoints
      int occ1 = 0, occ2 = 0, occ3 = 0;
      for (int i = 5; i < cloud_size - 5; ++i)
      {
        double depth1 = seg_info->segmentedCloudRange[i];
        double depth2 = seg_info->segmentedCloudRange[i + 1];
        int col_diff = abs(seg_info->segmentedCloudColInd[i] - seg_info->segmentedCloudColInd[i + 1]);
        if (col_diff < 10)
        {
          // TODO: 可以调下参数
          if (depth1 - depth2 > 0.5)
          {
            cloud_neighbor_picked_[i - 5] = cloud_neighbor_picked_[i - 4] = cloud_neighbor_picked_[i - 3] = cloud_neighbor_picked_[i - 2] = cloud_neighbor_picked_[i - 1] = cloud_neighbor_picked_[i] = true;
            occ1 += 6;
            continue;
          }
          else if (depth2 - depth1 > 0.5)
          {
            cloud_neighbor_picked_[i + 1] = cloud_neighbor_picked_[i + 2] = cloud_neighbor_picked_[i + 3] = cloud_neighbor_picked_[i + 4] = cloud_neighbor_picked_[i + 5] = true;
            occ2 += 5;
          }
        }
        double diff1 = abs(seg_info->segmentedCloudRange[i - 1] - depth1);
        double diff2 = abs(depth2 - depth1);
        if (diff1 > 0.02 * seg_info->segmentedCloudRange[i] && diff2 > 0.02 * seg_info->segmentedCloudRange[i])
        {
          cloud_neighbor_picked_[i] = true;
          ++occ3;
        }
      }
      NODELET_INFO("calculateSmoothness and markOccluded %.3f ms", t_adj.toc());
      NODELET_INFO("cloud_size: %d, occ1: %d, occ2: %d, occ3: %d", cloud_size, occ1, occ2, occ3);

      // step4: extractFeatures
      TicToc t_pts;
      PointCloudT::Ptr sharp(new PointCloudT);
      PointCloudT::Ptr less_sharp(new PointCloudT);
      PointCloudT::Ptr flat(new PointCloudT);
      PointCloudT::Ptr less_flat(new PointCloudT);
      PointCloudT::Ptr less_flat_scan(new PointCloudT);

      double t_sort = 0.;
      for (int i = 0; i < N_SCAN; ++i)
      {
        less_flat_scan->clear();
        for (int j = 0; j < 6; ++j)
        {
          int sp = (seg_info->startRingIndex[i] * (6 - j) + seg_info->endRingIndex[i] * j) / 6;
          int ep = (seg_info->startRingIndex[i] * (5 - j) + seg_info->endRingIndex[i] * (j + 1)) / 6 - 1;
          if (sp >= ep)
          {
            NODELET_WARN("sp: %d, ep: %d, start: %d, end: %d", sp, ep, seg_info->startRingIndex[i], seg_info->endRingIndex[i]);
            continue;
          }
          TicToc t_tmp;
          std::sort(cloud_sort_idx_.begin() + sp, cloud_sort_idx_.begin() + ep + 1, [this](int a, int b) { return cloud_curvature_[a] < cloud_curvature_[b]; });
          t_sort += t_tmp.toc();

          int picked_num = 0;
          for (int k = ep; k >= sp; --k)
          {
            int idx = cloud_sort_idx_[k];
            if (cloud_neighbor_picked_[idx] == 0 && cloud_curvature_[idx] > 0.1 && seg_info->segmentedCloudGroundFlag[idx] == false)
            {
              ++picked_num;
              cloud_neighbor_picked_[idx] = 1;
              if (picked_num <= 2)
              {
                cloud_label_[idx] = 2;
                sharp->push_back(seg_cloud->points[idx]);
                less_sharp->push_back(seg_cloud->points[idx]);
              }
              else if (picked_num <= 20)
              {
                cloud_label_[idx] = 1;
                less_sharp->push_back(seg_cloud->points[idx]);
              }
              else
              {
                break;
              }
              for (int l = 1; l <= 5; ++l)
              {
                int col_diff = abs(seg_info->segmentedCloudColInd[idx + l] - seg_info->segmentedCloudColInd[idx + l - 1]);
                if (col_diff > 10)
                {
                  break;
                }
                else
                {
                  cloud_neighbor_picked_[idx + l] = 1;
                }
              }
              for (int l = -1; l >= -5; --l)
              {
                int col_diff = abs(seg_info->segmentedCloudColInd[idx + l] - seg_info->segmentedCloudColInd[idx + l + 1]);
                if (col_diff > 10)
                {
                  break;
                }
                else
                {
                  cloud_neighbor_picked_[idx + l] = 1;
                }
              }
            }
          }

          picked_num = 0;
          for (int k = sp; k <= ep; ++k)
          {
            int idx = cloud_sort_idx_[k];
            if (cloud_neighbor_picked_[idx] == 0 && cloud_curvature_[idx] < 0.1 && seg_info->segmentedCloudGroundFlag[idx] == true)
            {
              cloud_label_[idx] = -1;
              flat->push_back(seg_cloud->points[idx]);
              ++picked_num;
              cloud_neighbor_picked_[idx] = 1;
              if (picked_num >= 4)
              {
                break;
              }
              for (int l = 1; l <= 5; ++l)
              {
                int col_diff = abs(seg_info->segmentedCloudColInd[idx + l] - seg_info->segmentedCloudColInd[idx + l - 1]);
                if (col_diff > 10)
                {
                  break;
                }
                else
                {
                  cloud_neighbor_picked_[idx + l] = 1;
                }
              }
              for (int l = -1; l >= -5; --l)
              {
                int col_diff = abs(seg_info->segmentedCloudColInd[idx + l] - seg_info->segmentedCloudColInd[idx + l + 1]);
                if (col_diff > 10)
                {
                  break;
                }
                else
                {
                  cloud_neighbor_picked_[idx + l] = 1;
                }
              }
            }
          }

          for (int k = sp; k <= ep; ++k)
          {
            if (cloud_label_[k] <= 0)
            {
              less_flat_scan->push_back(seg_cloud->points[k]);
            }
          }
        }

        PointCloudT::Ptr less_flat_scan_ds(new PointCloudT);
        pcl::VoxelGrid<PointT> ds;
        ds.setLeafSize(0.4, 0.4, 0.4);
        ds.setInputCloud(less_flat_scan);
        ds.filter(*less_flat_scan_ds);
        *less_flat += *less_flat_scan_ds;
      }
      NODELET_INFO("less_sharp size: %d, less_flat size: %d", less_sharp->size(), less_flat->size());
      NODELET_INFO("sort curvature value time: %.3f", t_sort);
      NODELET_INFO("feature prepare time: %.3f", t_pts.toc());

      if (pub_corner_.getNumSubscribers() > 0 || pub_corner_less_.getNumSubscribers() > 0 || pub_surf_.getNumSubscribers() > 0 || pub_surf_less_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2Ptr msg_sharp(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr msg_less_sharp(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr msg_flat(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr msg_less_flat(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*sharp, *msg_sharp);
        pcl::toROSMsg(*less_sharp, *msg_less_sharp);
        pcl::toROSMsg(*flat, *msg_flat);
        pcl::toROSMsg(*less_flat, *msg_less_flat);
        msg_sharp->header = msg_less_sharp->header = msg_flat->header = msg_less_flat->header = seg_cloud_buf_.front()->header;
        pub_corner_.publish(msg_sharp);
        pub_corner_less_.publish(msg_less_sharp);
        pub_surf_.publish(msg_flat);
        pub_surf_less_.publish(msg_less_flat);
      }

      if (system_initialized_ == false)
      {
        system_initialized_ = true;
        surf_last_ = less_flat;
        corner_last_ = less_sharp;
        kd_surf_last_->setInputCloud(surf_last_);
        kd_corner_last_->setInputCloud(corner_last_);
        NODELET_INFO("system initialized");
      }
      else
      {
        // ceres 优化
        TicToc t_opt;
        TicToc t_data;
        int corner_correspondance = 0, surf_correspondance = 0;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(params_, 6);
        std::vector<int> search_idx;
        std::vector<float> search_dist;
        for (int j = 0; j < flat->points.size(); ++j)
        {
          PointT point_sel;
          transformToStart(flat->points[j], point_sel);
          kd_surf_last_->nearestKSearch(point_sel, 1, search_idx, search_dist);
          int closest_idx = -1, min_idx2 = -1, min_idx3 = -1;
          if (search_dist[0] < nearest_feature_dist)
          {
            closest_idx = search_idx[0];
            double point_dist, min_dist2 = nearest_feature_dist, min_dist3 = nearest_feature_dist;
            int closest_scan = int(surf_last_->points[closest_idx].intensity);
            for (int k = closest_idx + 1; k < surf_last_->points.size(); ++k)
            {
              if (int(surf_last_->points[k].intensity) > closest_scan + 2.5)
              {
                break;
              }
              point_dist = pow(surf_last_->points[k].x - point_sel.x, 2) + pow(surf_last_->points[k].y - point_sel.y, 2) + pow(surf_last_->points[k].z - point_sel.z, 2);
              if (int(surf_last_->points[k].intensity) == closest_scan)
              {
                if (point_dist < min_dist2)
                {
                  min_dist2 = point_dist;
                  min_idx2 = k;
                }
              }
              else
              {
                if (point_dist < min_dist3)
                {
                  min_dist3 = point_dist;
                  min_idx3 = k;
                }
              }
            }
            for (int k = closest_idx - 1; k >= 0; --k)
            {
              if (int(surf_last_->points[k].intensity) < closest_scan - 2.5)
              {
                break;
              }
              point_dist = pow(surf_last_->points[k].x - point_sel.x, 2) + pow(surf_last_->points[k].y - point_sel.y, 2) + pow(surf_last_->points[k].z - point_sel.z, 2);
              if (int(surf_last_->points[k].intensity) == closest_scan)
              {
                if (point_dist < min_dist2)
                {
                  min_dist2 = point_dist;
                  min_idx2 = k;
                }
              }
              else
              {
                if (point_dist < min_dist3)
                {
                  min_dist3 = point_dist;
                  min_idx3 = k;
                }
              }
            }

            if (min_idx2 >= 0 && min_idx3 >= 0)
            {
              Eigen::Vector3d cp(flat->points[j].x, flat->points[j].y, flat->points[j].z);
              Eigen::Vector3d lpj(surf_last_->points[closest_idx].x, surf_last_->points[closest_idx].y, surf_last_->points[closest_idx].z);
              Eigen::Vector3d lpl(surf_last_->points[min_idx2].x, surf_last_->points[min_idx2].y, surf_last_->points[min_idx2].z);
              Eigen::Vector3d lpm(surf_last_->points[min_idx3].x, surf_last_->points[min_idx3].y, surf_last_->points[min_idx3].z);
              problem.AddResidualBlock(new SurfCostFunction(cp, lpj, lpl, lpm), loss_function, params_);
              ++surf_correspondance;
            }
          }
        }
        NODELET_INFO("surf correspondance: %d", surf_correspondance);
        NODELET_INFO("data association time: %.3fms", t_data.toc());
        if (surf_correspondance >= 10)
        {
          TicToc t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          options.max_num_iterations = 5;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          NODELET_INFO("solver time %f ms \n", t_solver.toc());
          NODELET_INFO_STREAM(summary.BriefReport());
        }
        else
        {
          NODELET_WARN("few surf correspondance");
        }
        t_data.tic();
        for (int j = 0; j < sharp->points.size(); ++j)
        {
          PointT point_sel;
          transformToStart(sharp->points[j], point_sel);
          kd_corner_last_->nearestKSearch(point_sel, 1, search_idx, search_dist);
          int closest_idx = -1, min_idx2 = -1;
          if (search_dist[0] < nearest_feature_dist)
          {
            closest_idx = search_idx[0];
            int closest_scan = int(corner_last_->points[closest_idx].intensity);
            double point_dist, min_point_dist2 = nearest_feature_dist;
            // lego 这里 k 的范围写的不对
            for (int k = closest_idx + 1; k < corner_last_->points.size(); ++k)
            {
              if (int(corner_last_->points[k].intensity) > closest_scan + 2)
              {
                break;
              }
              point_dist = pow(corner_last_->points[k].x - point_sel.x, 2) + pow(corner_last_->points[k].y - point_sel.y, 2) + pow(corner_last_->points[k].z - point_sel.z, 2);
              if (int(corner_last_->points[k].intensity) > closest_scan)
              {
                if (point_dist < min_point_dist2)
                {
                  min_point_dist2 = point_dist;
                  min_idx2 = k;
                }
              }
            }
            for (int k = closest_idx - 1; k >= 0; --k)
            {
              if (int(corner_last_->points[k].intensity) < closest_scan - 2)
              {
                break;
              }
              point_dist = pow(corner_last_->points[k].x - point_sel.x, 2) + pow(corner_last_->points[k].y - point_sel.y, 2) + pow(corner_last_->points[k].z - point_sel.z, 2);
              if (int(corner_last_->points[k].intensity) < closest_scan)
              {
                if (point_dist < min_point_dist2)
                {
                  min_point_dist2 = point_dist;
                  min_idx2 = k;
                }
              }
            }
          }
          if (min_idx2 >= 0)
          {
            Eigen::Vector3d cp(sharp->points[j].x, sharp->points[j].y, sharp->points[j].z);
            Eigen::Vector3d lpj(corner_last_->points[closest_idx].x, corner_last_->points[closest_idx].y, corner_last_->points[closest_idx].z);
            Eigen::Vector3d lpl(corner_last_->points[min_idx2].x, corner_last_->points[min_idx2].y, corner_last_->points[min_idx2].z);
            problem.AddResidualBlock(new CornerCostFunction(cp, lpj, lpl),
                                     loss_function, params_);
            ++corner_correspondance;
          }
        }
        NODELET_INFO("corner correspondance: %d", corner_correspondance);
        NODELET_INFO("data association time: %.3fms", t_data.toc());
        if (corner_correspondance >= 10)
        {
          TicToc t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          options.max_num_iterations = 5;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          NODELET_INFO("solver time %f ms \n", t_solver.toc());
          NODELET_INFO_STREAM(summary.BriefReport());
        }
        else
        {
          NODELET_WARN("few corner correspondance");
        }
        NODELET_INFO("total opt time: %.3fms", t_opt.toc());
        static double t_total = 0.;
        t_total += t_opt.toc();
        NODELET_INFO("opt time until now: %.3fms", t_total);
        Eigen::Vector3d t_last_cur(params_[0], params_[1], params_[2]);
        // 没有使用 roll, pitch
        Eigen::Matrix3d r_last_cur = (Eigen::AngleAxisd(params_[5], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        t_w_cur_ = t_w_cur_ + r_w_cur_ * t_last_cur;
        r_w_cur_ = r_w_cur_ * r_last_cur;
        NODELET_INFO("t_last_cur: %.3f, %.3f, %.3f, r_last_cur: %.3f, %.3f, %.3f", params_[0], params_[1], params_[2], params_[3], params_[4], params_[5]);
        NODELET_INFO_STREAM("t_w_cur: " << t_w_cur_ << "\nr_w_cur: " << r_w_cur_);

        // publish odometry
        Eigen::Quaterniond tmp_q(r_w_cur_);
        nav_msgs::OdometryPtr laser_odometry(new nav_msgs::Odometry);
        laser_odometry->header.frame_id = "/odom";
        laser_odometry->child_frame_id = "/laser";
        laser_odometry->header.stamp = ros::Time().fromSec(t1);
        laser_odometry->pose.pose.orientation.x = tmp_q.x();
        laser_odometry->pose.pose.orientation.y = tmp_q.y();
        laser_odometry->pose.pose.orientation.z = tmp_q.z();
        laser_odometry->pose.pose.orientation.w = tmp_q.w();
        laser_odometry->pose.pose.position.x = t_w_cur_.x();
        laser_odometry->pose.pose.position.y = t_w_cur_.y();
        laser_odometry->pose.pose.position.z = t_w_cur_.z();
        pub_odom_.publish(laser_odometry);

        tf::Transform tf_o2l;
        tf::poseMsgToTF(laser_odometry->pose.pose, tf_o2l);
        tf_broad_.sendTransform(tf::StampedTransform(tf_o2l, laser_odometry->header.stamp, "/odom", "/laser"));

        surf_last_ = less_flat;
        corner_last_ = less_sharp;
        kd_surf_last_->setInputCloud(surf_last_);
        kd_corner_last_->setInputCloud(corner_last_);
      }

      sensor_msgs::PointCloud2Ptr msg_surf_last(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2Ptr msg_corner_last(new sensor_msgs::PointCloud2);
      // sensor_msgs::PointCloud2Ptr msg_outlier_last(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*surf_last_, *msg_surf_last);
      pcl::toROSMsg(*corner_last_, *msg_corner_last);
      msg_surf_last->header.stamp.fromSec(t1);
      msg_surf_last->header.frame_id = "/laser";
      msg_corner_last->header = msg_surf_last->header;
      pub_surf_last_.publish(msg_surf_last);
      pub_corner_last_.publish(msg_corner_last);

      m_buf_.lock();
      seg_cloud_buf_.pop();
      seg_info_buf_.pop();
      outlier_buf_.pop();
      m_buf_.unlock();
    }
  }
}

void LaserOdometry::adjustDistortion(PointCloudT::Ptr cloud, double scan_time)
{
  int cloud_size = cloud->points.size();

  alego::cloud_infoConstPtr seg_info = seg_info_buf_.front();
  int start_ori = (seg_info->startOrientation + 2 * M_PI) / Horizon_SCAN;
  int end_ori = (seg_info->endOrientation + 2 * M_PI) / Horizon_SCAN;
  if (start_ori >= Horizon_SCAN)
  {
    start_ori -= Horizon_SCAN;
  }
  if (end_ori >= Horizon_SCAN)
  {
    end_ori -= Horizon_SCAN;
  }

  int ori_diff = end_ori - start_ori;
  if (ori_diff <= 0)
  {
    NODELET_WARN("ori_diff <= 0, %d", ori_diff);
    ori_diff = Horizon_SCAN;
  }
  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;

  for (int i = 0; i < cloud_size; ++i)
  {
    PointT &p = cloud->points[i];

    double rel_time = (seg_info->segmentedCloudColInd[i] - start_ori) * scan_period / ori_diff;
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
          NODELET_WARN_COND(i < 5, "unsync imu and pc msg %d, %d, %.6f, %.6f", imu_ptr_front_, imu_ptr_last_, cur_time, imu_time_[imu_ptr_front_]);
          return;
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
          double ratio_front = (cur_time - imu_time_[imu_ptr_back]) / (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
          double ratio_back = 1. - ratio_front;
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
          double ratio_front = (cur_time - odom_queue_[odom_ptr_back]->header.stamp.toSec()) / (odom_queue_[odom_ptr_front_]->header.stamp.toSec() - odom_queue_[odom_ptr_back]->header.stamp.toSec());
          double ratio_back = 1. - ratio_front;
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

void LaserOdometry::transformToStart(const PointT &pi, PointT &po)
{
  double s = 1;
  Eigen::Matrix3d r_point_last = (Eigen::AngleAxisd(params_[5] * s, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(params_[4] * s, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(params_[3] * s, Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Vector3d t_point_last(params_[0] * s, params_[1] * s, params_[2] * s);
  Eigen::Vector3d point(pi.x, pi.y, pi.z);
  Eigen::Vector3d un_point = r_point_last * point + t_point_last;

  po.x = un_point.x();
  po.y = un_point.y();
  po.z = un_point.z();
  po.intensity = pi.intensity;
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
  // imu_buf_.push(msg);
  double roll, pitch, yaw;
  tf::Quaternion ori;
  tf::quaternionMsgToTF(msg->orientation, ori);
  tf::Matrix3x3(ori).getRPY(roll, pitch, yaw);
  double acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  double acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  double acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_length;
  if ((imu_ptr_last_ + 1) % imu_queue_length == imu_ptr_front_)
  {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_length;
  }

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
  // odom_buf_.push(msg);

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
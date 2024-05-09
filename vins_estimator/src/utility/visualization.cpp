/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry, pub_latest_wheel_odometry, pub_latest_pure_wheel_odometry;
ros::Publisher pub_vins_path, pub_io_path;
ros::Publisher pub_groundtruth;
ros::Publisher pub_wheel_preintegration;
ros::Publisher pub_point_cloud, pub_margin_cloud, pub_margin_cloud2;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_vins_pose_marker;
ros::Publisher pub_io_pose_marker;
nav_msgs::Path vins_path;
nav_msgs::Path inertial_odometry_path;
nav_msgs::Path groundtruth_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher raw_track_image_publisher;
ros::Publisher semantic_track_image_publisher;
ros::Publisher pub_key_frame_feat;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
CameraPoseVisualization globIMUPose(0, 0, 1, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n) {
  pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
  pub_latest_wheel_odometry = n.advertise<nav_msgs::Odometry>("wheel_propagate", 1000);
  pub_latest_pure_wheel_odometry = n.advertise<nav_msgs::Odometry>("pure_wheel_propagate", 1000);
  pub_vins_path = n.advertise<nav_msgs::Path>("vins_path", 1000);
  pub_io_path = n.advertise<nav_msgs::Path>("inertial_odometry_path", 1000);
  pub_groundtruth = n.advertise<nav_msgs::Path>("groundtruth", 1000);
  pub_wheel_preintegration = n.advertise<nav_msgs::Path>("wheel_preintegration", 1000);
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
  pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
  pub_margin_cloud2 = n.advertise<sensor_msgs::PointCloud2>("margin_cloud2", 1000);
  pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
  pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
  pub_vins_pose_marker = n.advertise<visualization_msgs::MarkerArray>("vins_pose_marker", 1000);
  pub_io_pose_marker = n.advertise<visualization_msgs::MarkerArray>("io_pose_marker", 1000);
  pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
  pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
  pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
  raw_track_image_publisher = n.advertise<sensor_msgs::Image>("raw_track_image", 1000);
  semantic_track_image_publisher = n.advertise<sensor_msgs::Image>("semantic_track_image", 1000);
  pub_key_frame_feat = n.advertise<sensor_msgs::Image>("image_feat", 1000);

  cameraposevisual.setScale(0.1);
  cameraposevisual.setLineWidth(0.01);

  globIMUPose.setScale(0.1);
  globIMUPose.setLineWidth(0.01);
}

void pubLatestInertialOdometry(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                               const Eigen::Vector3d &velocity, double t) {
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(t);
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
  pub_latest_odometry.publish(odometry);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time(t);
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose = odometry.pose.pose;
  inertial_odometry_path.header.stamp = ros::Time(t);
  inertial_odometry_path.header.frame_id = "world";
  inertial_odometry_path.poses.push_back(pose_stamped);
  pub_io_path.publish(inertial_odometry_path);
  if (inertial_odometry_path.poses.size() > 500) {
    inertial_odometry_path.poses.erase(inertial_odometry_path.poses.begin());
  }

  globIMUPose.reset();
  globIMUPose.add_pose(position, orientation);
  // globIMUPose.publish_by(pub_io_pose_marker, odometry.header);
}

void pubLatestWheelOdometry(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                            const Eigen::Vector3d &velocity, double t) {
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(t);
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
  pub_latest_wheel_odometry.publish(odometry);
}

void pubPureWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &orientation,
                                const Eigen::Vector3d &velocity, double t) {
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(t);
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = P.x();
  odometry.pose.pose.position.y = P.y();
  odometry.pose.pose.position.z = P.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
  pub_latest_pure_wheel_odometry.publish(odometry);
}

void publishRawTrackImage(const cv::Mat &imgTrack, const double t) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
  raw_track_image_publisher.publish(imgTrackMsg);
}

void publishSemanticTrackImage(const cv::Mat &imgTrack, const double t) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "mono8", imgTrack).toImageMsg();
  semantic_track_image_publisher.publish(imgTrackMsg);
}

void printStatistics(const Estimator &estimator, double t) {
  if (estimator.solver_flag != Estimator::SolverFlag::VIO) return;
  ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
  ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
  if (ESTIMATE_EXTRINSIC || ESTIMATE_EXTRINSIC_WHEEL || USE_PLANE) {
    cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
    if (ESTIMATE_EXTRINSIC) {
      for (int i = 0; i < NUM_OF_CAM; i++) {
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

        Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
        eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
        eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
        cv::Mat cv_T;
        cv::eigen2cv(eigen_T, cv_T);
        if (i == 0)
          fs << "body_T_cam0" << cv_T;
        else
          fs << "body_T_cam1" << cv_T;
      }
    }

    if (ESTIMATE_EXTRINSIC_WHEEL) {
      ROS_DEBUG_STREAM("extirnsic tio: " << estimator.tio.transpose());
      ROS_DEBUG_STREAM("extrinsic rio: " << Utility::R2ypr(estimator.rio).transpose());

      Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
      eigen_T.block<3, 3>(0, 0) = estimator.rio;
      eigen_T.block<3, 1>(0, 3) = estimator.tio;
      cv::Mat cv_T;
      cv::eigen2cv(eigen_T, cv_T);
      fs << "body_T_wheel" << cv_T;
    }

    if (USE_PLANE) {
      ROS_DEBUG_STREAM("plane zpw: " << estimator.zpw);
      ROS_DEBUG_STREAM("plane rpw: " << Utility::R2ypr(estimator.rpw).transpose());

      Eigen::Matrix3d eigen_T = estimator.rpw;
      cv::Mat cv_T;
      cv::eigen2cv(eigen_T, cv_T);
      fs << "plane_R_world" << cv_T;
      fs << "plane_Z_world" << estimator.zpw;
    }

    fs.release();
  }
  if (ESTIMATE_INTRINSIC_WHEEL) {
    cv::FileStorage fs(IN_CALIB_RESULT_PATH, cv::FileStorage::WRITE);

    if (ESTIMATE_INTRINSIC_WHEEL) {
      ROS_DEBUG("intirnsic sx: %f,  sy: %f,  sw: %f", estimator.sx, estimator.sy, estimator.sw);
      fs << "sx" << estimator.sx;
      fs << "sy" << estimator.sy;
      fs << "sw" << estimator.sw;
    }
  }

  static double sum_of_time = 0;
  static int sum_of_calculation = 0;
  sum_of_time += t;
  sum_of_calculation++;
  ROS_DEBUG("vo solver costs: %f ms", t);
  ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

  sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
  last_path = estimator.Ps[WINDOW_SIZE];
  ROS_DEBUG("sum of path %f", sum_of_path);
  if (ESTIMATE_TD_INERTIAL) {
    ROS_INFO("td_inertial %f", estimator.td_inertial);
    std::ofstream ofs(TD_PATH, ios::app);
    if (!ofs.is_open()) {
      ROS_WARN("cannot open %s", TD_PATH.c_str());
    }
    ofs << estimator.td_inertial << std::endl;
  }

  if (ESTIMATE_TD_WHEEL) {
    ROS_INFO("td_wheel %f", estimator.td_wheel);
    std::ofstream ofs(TD_WHEEL_PATH, ios::app);
    if (!ofs.is_open()) {
      ROS_WARN("cannot open %s", TD_WHEEL_PATH.c_str());
    }
    ofs << estimator.td_wheel << std::endl;
  }
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header) {
  if (estimator.solver_flag == Estimator::SolverFlag::VIO) {
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    Quaterniond tmp_Q;
    tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
    odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    odometry.pose.pose.orientation.x = tmp_Q.x();
    odometry.pose.pose.orientation.y = tmp_Q.y();
    odometry.pose.pose.orientation.z = tmp_Q.z();
    odometry.pose.pose.orientation.w = tmp_Q.w();
    odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    vins_path.header = header;
    vins_path.header.frame_id = "world";
    vins_path.poses.push_back(pose_stamped);
    pub_vins_path.publish(vins_path);

    ofstream foutC(VINS_RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC << std::setprecision(9) << header.stamp.toSec() << " " << std::setprecision(9) << pose_stamped.pose.position.x
          << " " << pose_stamped.pose.position.y << " " << pose_stamped.pose.position.z << " "
          << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " "
          << pose_stamped.pose.orientation.z << " " << pose_stamped.pose.orientation.w << std::endl;
    auto tmp_T = pose_stamped.pose.position;
  }
}

void pubGroundTruth(Estimator &estimator, const std_msgs::Header &header, Eigen::Matrix<double, 7, 1> &pose,
                    const double td) {
  estimator.mGTBuf.lock();
  if (estimator.groundtruthBuf.empty()) {
    estimator.mGTBuf.unlock();
    return;
  }
  double groundtruth_time = estimator.groundtruthBuf.front().first;
  double header_time = header.stamp.toSec() - OFFSET_SIM;
  pose = estimator.groundtruthBuf.front().second;
  while (groundtruth_time < (header_time - 1e-5)) {
    estimator.groundtruthBuf.pop();
    if (estimator.groundtruthBuf.empty()) break;
    groundtruth_time = estimator.groundtruthBuf.front().first;
    pose = estimator.groundtruthBuf.front().second;
  }
  if (!estimator.groundtruthBuf.empty() && groundtruth_time > (header_time + 1e-5)) {
    ROS_INFO("wait for new frame");
    ROS_INFO("groundtruth_time: %f, header_time: %f", groundtruth_time, header_time);
    estimator.mGTBuf.unlock();
    return;
  }
  if (estimator.groundtruthBuf.empty() || abs(groundtruth_time - header_time) > 1e-5) {
    ROS_ERROR("can not find corresponding groundtruth");
  }
  estimator.mGTBuf.unlock();

  if (estimator.solver_flag == Estimator::SolverFlag::VIO) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = pose[0];
    pose_stamped.pose.orientation.x = pose[1];
    pose_stamped.pose.orientation.y = pose[2];
    pose_stamped.pose.orientation.z = pose[3];
    pose_stamped.pose.position.x = pose[4];
    pose_stamped.pose.position.y = pose[5];
    pose_stamped.pose.position.z = pose[6];
    groundtruth_path.header = header;
    groundtruth_path.header.frame_id = "world";
    groundtruth_path.poses.push_back(pose_stamped);
    pub_groundtruth.publish(groundtruth_path);

    ofstream foutC(GROUNDTRUTH_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC << std::setprecision(0) << header.stamp.toSec() * 1e9 << " " << std::setprecision(9)
          << pose_stamped.pose.position.x << " " << pose_stamped.pose.position.y << " " << pose_stamped.pose.position.z
          << " " << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " "
          << pose_stamped.pose.orientation.z << " " << pose_stamped.pose.orientation.w << std::endl;

    auto tmp_T = pose_stamped.pose.position;
    auto tmp_Q = pose_stamped.pose.orientation;
    printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x, tmp_T.y, tmp_T.z, tmp_Q.w, tmp_Q.x,
           tmp_Q.y, tmp_Q.z);
  }
}

void pubWheelPreintegration(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                            const std_msgs::Header &header) {
  static nav_msgs::Path preintegration_path;
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.orientation.w = orientation.w();
  pose_stamped.pose.orientation.x = orientation.x();
  pose_stamped.pose.orientation.y = orientation.y();
  pose_stamped.pose.orientation.z = orientation.z();
  pose_stamped.pose.position.x = position[0];
  pose_stamped.pose.position.y = position[1];
  pose_stamped.pose.position.z = position[2];
  preintegration_path.header = header;
  preintegration_path.header.frame_id = "world";
  preintegration_path.poses.push_back(pose_stamped);
  pub_wheel_preintegration.publish(preintegration_path);
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header) {
  if (estimator.key_poses.size() == 0) return;
  visualization_msgs::Marker key_poses;
  key_poses.header = header;
  key_poses.header.frame_id = "world";
  key_poses.ns = "key_poses";
  key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
  key_poses.action = visualization_msgs::Marker::ADD;
  key_poses.pose.orientation.w = 1.0;
  key_poses.lifetime = ros::Duration();

  key_poses.id = 0;
  key_poses.scale.x = 0.05;
  key_poses.scale.y = 0.05;
  key_poses.scale.z = 0.05;
  key_poses.color.r = 1.0;
  key_poses.color.a = 1.0;

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    geometry_msgs::Point pose_marker;
    Vector3d correct_pose;
    correct_pose = estimator.key_poses[i];
    pose_marker.x = correct_pose.x();
    pose_marker.y = correct_pose.y();
    pose_marker.z = correct_pose.z();
    key_poses.points.push_back(pose_marker);
  }
  pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header) {
  int idx2 = WINDOW_SIZE - 1;

  if (estimator.solver_flag == Estimator::SolverFlag::VIO) {
    int i = idx2;
    Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();

    pub_camera_pose.publish(odometry);

    cameraposevisual.reset();
    cameraposevisual.add_pose(P, R);
    if (STEREO) {
      Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
      Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
      cameraposevisual.add_pose(P, R);
    }
    cameraposevisual.publish_by(pub_vins_pose_marker, odometry.header);
  }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header) {
  sensor_msgs::PointCloud point_cloud, loop_point_cloud;
  point_cloud.header = header;
  loop_point_cloud.header = header;

  for (auto &it_per_id : estimator.f_manager.feature) {
    int used_num;
    used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) continue;
    if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1) continue;
    int imu_i = it_per_id.start_frame;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

    geometry_msgs::Point32 p;
    p.x = w_pts_i(0);
    p.y = w_pts_i(1);
    p.z = w_pts_i(2);

    if (it_per_id.weight < 0.1) continue;
    point_cloud.points.push_back(p);
  }
  ROS_DEBUG("good point size: %d", point_cloud.points.size());
  pub_point_cloud.publish(point_cloud);

  pcl::PointCloud<pcl::PointXYZI> margin_ptcl;
  sensor_msgs::PointCloud margin_cloud;
  margin_cloud.header = header;

  for (auto &it_per_id : estimator.f_manager.feature) {
    if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1) {
      int imu_i = it_per_id.start_frame;
      Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
      Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

      pcl::PointXYZI margin_pt;
      margin_pt.x = (float)w_pts_i(0);
      margin_pt.y = (float)w_pts_i(1);
      margin_pt.z = (float)w_pts_i(2);
      margin_pt.intensity = (float)it_per_id.weight;
      margin_ptcl.push_back(margin_pt);

      if (it_per_id.weight < 0.1) continue;
      geometry_msgs::Point32 p;
      p.x = w_pts_i(0);
      p.y = w_pts_i(1);
      p.z = w_pts_i(2);
      margin_cloud.points.push_back(p);
    }
  }
  if (margin_ptcl.empty()) return;
  pub_margin_cloud.publish(margin_cloud);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(margin_ptcl, output);
  output.header = margin_cloud.header;
  pub_margin_cloud2.publish(output);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header) {
  if (estimator.solver_flag != Estimator::SolverFlag::VIO) return;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  Vector3d correct_t;
  Quaterniond correct_q;
  correct_t = estimator.Ps[WINDOW_SIZE];
  correct_q = estimator.Rs[WINDOW_SIZE];

  transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

  transform.setOrigin(tf::Vector3(estimator.tic[0].x(), estimator.tic[0].y(), estimator.tic[0].z()));
  q.setW(Quaterniond(estimator.ric[0]).w());
  q.setX(Quaterniond(estimator.ric[0]).x());
  q.setY(Quaterniond(estimator.ric[0]).y());
  q.setZ(Quaterniond(estimator.ric[0]).z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

  if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) {
    transform.setOrigin(tf::Vector3(estimator.tio.x(), estimator.tio.y(), estimator.tio.z()));
    q.setW(Quaterniond(estimator.rio).w());
    q.setX(Quaterniond(estimator.rio).x());
    q.setY(Quaterniond(estimator.rio).y());
    q.setZ(Quaterniond(estimator.rio).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "wheel"));
  }

  if (USE_PLANE) {
    transform.setOrigin(tf::Vector3(0, 0, estimator.zpw));
    q.setW(Quaterniond(estimator.rpw).w());
    q.setX(Quaterniond(estimator.rpw).x());
    q.setY(Quaterniond(estimator.rpw).y());
    q.setZ(Quaterniond(estimator.rpw).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "plane", "world"));
    ROS_DEBUG_STREAM("plane rpw: " << Eigen::Quaterniond(estimator.rpw).coeffs().transpose()
                                   << " zpw: " << estimator.zpw);
  }

  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = estimator.tic[0].x();
  odometry.pose.pose.position.y = estimator.tic[0].y();
  odometry.pose.pose.position.z = estimator.tic[0].z();
  Quaterniond tmp_q{estimator.ric[0]};
  odometry.pose.pose.orientation.x = tmp_q.x();
  odometry.pose.pose.orientation.y = tmp_q.y();
  odometry.pose.pose.orientation.z = tmp_q.z();
  odometry.pose.pose.orientation.w = tmp_q.w();
  pub_extrinsic.publish(odometry);
}

void pubKeyframe(Estimator &estimator) {
  static int idx = 0;
  if (estimator.solver_flag == Estimator::SolverFlag::VIO && estimator.marginalization_flag == 0) {
    int i = WINDOW_SIZE - 2;
    cv::Mat saveImg;
    double t;

    if (SHOW_IMAGE_WEIGHT) {
      while (estimator.imageBuf_0.front().first < estimator.Headers[i]) {
        estimator.imageBuf_0.pop();
      }
      if (estimator.imageBuf_0.empty() || estimator.imageBuf_0.front().first != estimator.Headers[i]) return;
      saveImg = estimator.imageBuf_0.front().second.clone();
      t = estimator.imageBuf_0.front().first;
      cv::cvtColor(saveImg, saveImg, CV_GRAY2RGB);
    }

    Vector3d P = estimator.Ps[i];
    Quaterniond R = Quaterniond(estimator.Rs[i]);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();

    pub_keyframe_pose.publish(odometry);

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
    point_cloud.header.frame_id = "world";

    cv::Mat featureImg = saveImg.clone();

    for (auto &it_per_id : estimator.f_manager.feature) {
      int frame_size = it_per_id.feature_per_frame.size();
      if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 &&
          it_per_id.solve_flag == 1) {
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);

        int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
        sensor_msgs::ChannelFloat32 p_2d;
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
        p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
        p_2d.values.push_back(it_per_id.feature_id);
        point_cloud.channels.push_back(p_2d);
        if (SHOW_IMAGE_WEIGHT) {
          cv::Scalar color(0, 255 * it_per_id.weight, 255 * (1 - it_per_id.weight));
          cv::Point pt =
              cv::Point(it_per_id.feature_per_frame[imu_j].uv.x(), it_per_id.feature_per_frame[imu_j].uv.y());
          cv::circle(featureImg, pt, 3, color, -1);
        }
      }
    }

    if (SHOW_IMAGE_WEIGHT) {
      std_msgs::Header header;
      header.frame_id = "world";
      header.stamp = ros::Time(estimator.Headers[i]);
      sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", featureImg).toImageMsg();
      pub_key_frame_feat.publish(imgTrackMsg);
    }

    pub_keyframe_point.publish(point_cloud);
    idx++;
  }
}
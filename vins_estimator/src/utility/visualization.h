/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include "CameraPoseVisualization.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_vins_path, pub_pose, pub_io_path;
extern ros::Publisher pub_groundtruth;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key_frame_feat;
extern ros::Publisher pub_key;
extern nav_msgs::Path vins_path;
extern nav_msgs::Path inertial_odometry_path;
extern nav_msgs::Path groundtruth_path;
extern ros::Publisher pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

void pubLatestInertialOdometry(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                               const Eigen::Vector3d &velocity, double t);

void pubLatestWheelOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubPureWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V,
                                double t);

void publishRawTrackImage(const cv::Mat &imgTrack, const double t);

void publishSemanticTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

void pubGroundTruth(Estimator &estimator, const std_msgs::Header &header, Eigen::Matrix<double, 7, 1> &pose,
                    const double td);

void pubWheelPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::Header &header);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator &estimator, const std_msgs::Header &header);

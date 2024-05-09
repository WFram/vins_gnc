/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include "../estimator/feature_manager.h"
#include "../factor/imu_factor.h"
#include "../factor/wheel_integration_base.h"
#include "../utility/utility.h"

using namespace Eigen;
using namespace std;

class ImageFrame {
 public:
  ImageFrame(){};

  ImageFrame(const FeatureFrame &input_points, double input_timestamp)
      : timestamp{input_timestamp}, is_key_frame{false} {
    points = input_points;
  };

  FeatureFrame points;
  double timestamp;
  Matrix3d rotation;
  Vector3d translation;
  IntegrationBase *imu_preintegration;
  WheelIntegrationBase *pre_integration_wheel;
  bool is_key_frame;
};

void solveGyroscopeBias(map<double, ImageFrame> &image_frame_history, Vector3d *Bgs);

bool LinearIMUAlignment(map<double, ImageFrame> &image_frame_history, Vector3d &gravity_vector, VectorXd &x);

bool LinearIMUWheelAlignment(map<double, ImageFrame> &image_frame_history, Vector3d &gravity_vector, VectorXd &x);

bool visualAlignment(map<double, ImageFrame> &image_frame_history, Vector3d *Bgs, Vector3d &gravity_vector, VectorXd &x);
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include <ceres/ceres.h>
#include "../estimator/parameters.h"
#include "../utility/sophus_utils.hpp"
#include "../utility/utility.h"
#include "wheel_integration_base.h"

class OriginPlaneFactor : public ceres::SizedCostFunction<3, 7> {
 public:
  OriginPlaneFactor(){};
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    Eigen::Vector3d e3 = Eigen::Vector3d(0, 0, 1);
    Eigen::Matrix3d gam = Eigen::Matrix3d::Identity();
    residual.block<2, 1>(0, 0) = gam.block<2, 3>(0, 0) * Qi.toRotationMatrix().transpose() * e3;
    residual[2] = e3.transpose() * Pi;

    Eigen::Matrix<double, 3, 3> sqrt_info = Eigen::Vector3d(PITCH_N_INV, ROLL_N_INV, ZPW_N_INV).asDiagonal();
    residual = sqrt_info * residual;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        jacobian_pose_i.block<2, 3>(0, O_R) =
            gam.block<2, 3>(0, 0) * Utility::skewSymmetric(Qi.toRotationMatrix().transpose() * e3);
        jacobian_pose_i.block<1, 3>(2, O_P) = e3.transpose();

        jacobian_pose_i = sqrt_info * jacobian_pose_i;

        if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
          ROS_WARN("numerical unstable in preintegration");
        }
      }
    }

    return true;
  }
};

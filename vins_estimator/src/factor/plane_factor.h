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

class PlaneFactor : public ceres::SizedCostFunction<3, 7, 7, 4, 1> {
 public:
  PlaneFactor(){};
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tio(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qio(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Quaterniond qpw(parameters[2][3], parameters[2][0], parameters[2][1], parameters[2][2]);

    double zpw = parameters[3][0];

#if 0
        if ((Bai - preintegration->linearized_ba).norm() > 0.10 ||
            (Bgi - preintegration->linearized_bg).norm() > 0.01)
        {
            preintegration->repropagate(Bai, Bgi);
        }
#endif

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    Eigen::Vector3d e3 = Eigen::Vector3d(0, 0, 1);
    Eigen::Matrix3d gam = Eigen::Matrix3d::Identity();
    Eigen::Vector3d toi = -tio;
    residual.block<2, 1>(0, 0) = gam.block<2, 3>(0, 0) * qio.toRotationMatrix().transpose() *
                                 Qi.toRotationMatrix().transpose() * qpw.toRotationMatrix().transpose() * e3;
    residual[2] = zpw + (qpw * (Pi + Qi * tio))[2];

    Eigen::Matrix<double, 3, 3> sqrt_info = Eigen::Vector3d(PITCH_N_INV, ROLL_N_INV, ZPW_N_INV).asDiagonal();
    residual = sqrt_info * residual;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        jacobian_pose_i.block<2, 3>(0, O_R) = gam.block<2, 3>(0, 0) * qio.toRotationMatrix().transpose() *
                                              Utility::skewSymmetric(Qi.inverse() * qpw.inverse() * e3);
        jacobian_pose_i.block<1, 3>(2, O_P) = e3.transpose() * qpw.toRotationMatrix();
        jacobian_pose_i.block<1, 3>(2, O_R) =
            -e3.transpose() * qpw.toRotationMatrix() * Qi.toRotationMatrix() * Utility::skewSymmetric(tio);

        jacobian_pose_i = sqrt_info * jacobian_pose_i;

        if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
          ROS_WARN("numerical unstable in preintegration");
        }
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
        jacobian_ex_pose.setZero();

        jacobian_ex_pose.block<2, 3>(0, O_R) =
            gam.block<2, 3>(0, 0) * Utility::skewSymmetric(qio.inverse() * Qi.inverse() * qpw.inverse() * e3);
        jacobian_ex_pose.block<1, 3>(2, O_P) = e3.transpose() * qpw.toRotationMatrix() * Qi.toRotationMatrix();

        jacobian_ex_pose = sqrt_info * jacobian_ex_pose;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_qpw(jacobians[2]);
        jacobian_qpw.setZero();

        jacobian_qpw.block<2, 3>(0, 0) = gam.block<2, 3>(0, 0) * qio.toRotationMatrix().transpose() *
                                         Qi.toRotationMatrix().transpose() * Utility::skewSymmetric(qpw.inverse() * e3);
        jacobian_qpw.block<1, 3>(2, 0) =
            -e3.transpose() * qpw.toRotationMatrix() * Utility::skewSymmetric(Pi + Qi * tio);

        jacobian_qpw = sqrt_info * jacobian_qpw;
      }

      if (jacobians[3]) {
        Eigen::Map<Eigen::Vector3d> jacobian_zpw(jacobians[3]);
        jacobian_zpw.setZero();

        jacobian_zpw[2] = 1;

        jacobian_zpw = sqrt_info * jacobian_zpw;
      }
    }

    return true;
  }
};

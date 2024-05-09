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

#include "../estimator/parameters.h"
#include "../utility/utility.h"
#include "integration_base.h"

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include "pose_local_parameterization.h"

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
 public:
  IMUFactor() = delete;

  IMUFactor(IntegrationBase *input_preintegration) : preintegration(input_preintegration) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

#if 0
        if ((Bai - preintegration->linearized_ba).norm() > 0.10 ||
            (Bgi - preintegration->linearized_bg).norm() > 0.01)
        {
            preintegration->repropagate(Bai, Bgi);
        }
#endif

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = preintegration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);

    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(preintegration->covariance.inverse()).matrixL().transpose();

    residual = sqrt_info * residual;

    if (jacobians) {
      double sum_dt = preintegration->sum_dt;
      Eigen::Matrix3d dp_dba = preintegration->jacobian.template block<3, 3>(O_P, O_BA);
      Eigen::Matrix3d dp_dbg = preintegration->jacobian.template block<3, 3>(O_P, O_BG);

      Eigen::Matrix3d dq_dbg = preintegration->jacobian.template block<3, 3>(O_R, O_BG);

      Eigen::Matrix3d dv_dba = preintegration->jacobian.template block<3, 3>(O_V, O_BA);
      Eigen::Matrix3d dv_dbg = preintegration->jacobian.template block<3, 3>(O_V, O_BG);

      if (preintegration->jacobian.maxCoeff() > 1e8 || preintegration->jacobian.minCoeff() < -1e8) {
        ROS_WARN("IMU preintegration is numerically unstable");
      }

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_reference_pose(jacobians[0]);
        jacobian_reference_pose.setZero();

        jacobian_reference_pose.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
        jacobian_reference_pose.block<3, 3>(O_P, O_R) =
            Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
            jacobian_reference_pose.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
        Eigen::Quaterniond corrected_preintegration_gamma =
            preintegration->preintegration_gamma *
            Utility::deltaQ(dq_dbg * (Bgi - preintegration->linearized_gyro_bias));
        jacobian_reference_pose.block<3, 3>(O_R, O_R) =
            -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_preintegration_gamma))
                 .bottomRightCorner<3, 3>();
#endif

        jacobian_reference_pose.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

        jacobian_reference_pose = sqrt_info * jacobian_reference_pose;

        if (jacobian_reference_pose.maxCoeff() > 1e8 || jacobian_reference_pose.minCoeff() < -1e8) {
          ROS_WARN("IMU preintegration is numerically unstable");
        }
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_reference_velocity_bias(jacobians[1]);
        jacobian_reference_velocity_bias.setZero();
        jacobian_reference_velocity_bias.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_reference_velocity_bias.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
        jacobian_reference_velocity_bias.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
            jacobian_reference_velocity_bias.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
        jacobian_reference_velocity_bias.block<3, 3>(O_R, O_BG - O_V) =
            -Utility::Qleft(Qj.inverse() * Qi * preintegration->preintegration_gamma).bottomRightCorner<3, 3>() *
            dq_dbg;
#endif

        jacobian_reference_velocity_bias.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
        jacobian_reference_velocity_bias.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
        jacobian_reference_velocity_bias.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

        jacobian_reference_velocity_bias.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_reference_velocity_bias.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_reference_velocity_bias = sqrt_info * jacobian_reference_velocity_bias;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_target_pose(jacobians[2]);
        jacobian_target_pose.setZero();

        jacobian_target_pose.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
            jacobian_target_pose.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
        Eigen::Quaterniond corrected_preintegration_gamma =
            preintegration->preintegration_gamma *
            Utility::deltaQ(dq_dbg * (Bgi - preintegration->linearized_gyro_bias));
        jacobian_target_pose.block<3, 3>(O_R, O_R) =
            Utility::Qleft(corrected_preintegration_gamma.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

        jacobian_target_pose = sqrt_info * jacobian_target_pose;
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_target_velocity_bias(jacobians[3]);
        jacobian_target_velocity_bias.setZero();

        jacobian_target_velocity_bias.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

        jacobian_target_velocity_bias.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

        jacobian_target_velocity_bias.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

        jacobian_target_velocity_bias = sqrt_info * jacobian_target_velocity_bias;
      }
    }

    return true;
  }

  IntegrationBase *preintegration;
};

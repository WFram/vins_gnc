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

class WheelFactor : public ceres::SizedCostFunction<6, 7, 7, 7, 1, 1, 1, 1> {
 public:
  WheelFactor() = delete;

  WheelFactor(WheelIntegrationBase *input_preintegration) : preintegration(input_preintegration) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d Pio(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qio(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double sx = parameters[3][0];
    double sy = parameters[4][0];
    double sw = parameters[5][0];

    Eigen::Matrix3d sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
    double td = parameters[6][0];

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual = preintegration->evaluate(Pi, Qi, Qio, Pio, sx, sy, sw, Pj, Qj, td);

    Eigen::Matrix<double, 6, 1> raw_residual = residual;

    Eigen::Matrix<double, 6, 6> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(preintegration->covariance.inverse()).matrixL().transpose();
    residual = sqrt_info * residual;

    if (jacobians) {
      Eigen::Vector3d dp_dsx = preintegration->jacobian.template block<3, 1>(O_P, 0);
      Eigen::Vector3d dp_dsy = preintegration->jacobian.template block<3, 1>(O_P, 1);
      Eigen::Vector3d dp_dsw = preintegration->jacobian.template block<3, 1>(O_P, 2);
      Eigen::Vector3d dq_dsw = preintegration->jacobian.template block<3, 1>(O_R, 2);
      double dtd = td - preintegration->linearized_td;
      if (preintegration->jacobian.maxCoeff() > 1e8 || preintegration->jacobian.minCoeff() < -1e8) {
        ROS_WARN("Wheel odometry preintegration is numerically unstable");
      }

      Eigen::Vector3d raw_residual_theta = raw_residual.block<3, 1>(3, 0);
      Eigen::Matrix3d Jr_delta_theta_inv;
      Sophus::rightJacobianInvSO3(raw_residual_theta, Jr_delta_theta_inv);

      Eigen::Vector3d dr_dsw = dq_dsw * (sw - preintegration->linearized_sw);
      Eigen::Matrix3d Jr_dr_dsw;
      Sophus::rightJacobianSO3(dr_dsw, Jr_dr_dsw);

      Eigen::Matrix3d Ri = Qi.toRotationMatrix();
      Eigen::Matrix3d Rj = Qj.toRotationMatrix();
      Eigen::Matrix3d Rio = Qio.toRotationMatrix();
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_reference_pose(jacobians[0]);
        jacobian_reference_pose.setZero();

        jacobian_reference_pose.block<3, 3>(O_P, O_P) = -(Qi * Qio).inverse().toRotationMatrix();
        jacobian_reference_pose.block<3, 3>(O_P, O_R) =
            (Ri * Rio).transpose() * (Ri * Utility::skewSymmetric(Pio)) +
            Rio.transpose() * Utility::skewSymmetric(Ri.transpose() * (Rj * Pio + Pj - Ri * Pio - Pi));

#if 0
            jacobian_reference_pose.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else

        jacobian_reference_pose.block<3, 3>(O_R, O_R) =
            -Jr_delta_theta_inv * ((Qj * Qio).inverse() * Qi).toRotationMatrix();
#endif

        jacobian_reference_pose = sqrt_info * jacobian_reference_pose;

        if (jacobian_reference_pose.maxCoeff() > 1e8 || jacobian_reference_pose.minCoeff() < -1e8) {
          ROS_WARN("Wheel odometry preintegration is numerically unstable");
        }
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_target_pose(jacobians[1]);
        jacobian_target_pose.setZero();

        jacobian_target_pose.block<3, 3>(O_P, O_P) = (Qi * Qio).inverse().toRotationMatrix();
        jacobian_target_pose.block<3, 3>(O_P, O_R) =
            -((Qi * Qio).inverse() * Qj).toRotationMatrix() * Utility::skewSymmetric(Pio);

#if 0
            jacobian_target_pose.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
        jacobian_target_pose.block<3, 3>(O_R, O_R) = Jr_delta_theta_inv * (Qio.inverse()).toRotationMatrix();
#endif

        jacobian_target_pose = sqrt_info * jacobian_target_pose;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_extrinsic(jacobians[2]);
        jacobian_extrinsic.setZero();

        jacobian_extrinsic.block<3, 3>(O_P, O_P) =
            (Qi * Qio).inverse().toRotationMatrix() * (Qj.toRotationMatrix() - Qi.toRotationMatrix());

        jacobian_extrinsic.block<3, 3>(O_P, O_R) =
            Utility::skewSymmetric((Qi * Qio).inverse() * (Qj * Pio + Pj - Qi * Pio - Pi));

        jacobian_extrinsic.block<3, 3>(O_R, O_R) =
            Jr_delta_theta_inv * (Eigen::Matrix3d::Identity() - ((Qj * Qio).inverse() * Qi * Qio).toRotationMatrix());

        jacobian_extrinsic = sqrt_info * jacobian_extrinsic;
      }

      Eigen::Vector3d forward_propagated_sw = sw * preintegration->linearized_angular_velocity * dtd;
      Eigen::Vector3d forward_propagated_sv = sv * preintegration->linearized_linear_velocity * dtd;
      Eigen::Vector3d back_propagated_sw = sw * preintegration->input_angular_velocity * dtd;
      Eigen::Vector3d back_propagated_sv = sv * preintegration->input_linear_velocity * dtd;

      Eigen::Matrix3d Jr_positive_td;
      Eigen::Matrix3d Jr_negative_td;
      Sophus::rightJacobianSO3(forward_propagated_sw, Jr_positive_td);
      Sophus::rightJacobianSO3(-forward_propagated_sw, Jr_negative_td);
      Eigen::Matrix3d unit_x_diagonal_matrix = Eigen::Vector3d(1, 0, 0).asDiagonal();
      Eigen::Matrix3d unit_y_diagonal_matrix = Eigen::Vector3d(0, 1, 0).asDiagonal();

      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_sx(jacobians[3]);
        jacobian_sx.setZero();

        jacobian_sx.block<3, 1>(O_P, 0) =
            -Sophus::SO3d::exp(forward_propagated_sv).matrix() *
            (unit_x_diagonal_matrix * preintegration->linearized_linear_velocity * dtd + dp_dsx -
             preintegration->corrected_preintegration_gamma.toRotationMatrix() * unit_x_diagonal_matrix *
                 preintegration->input_linear_velocity * dtd);

        jacobian_sx = sqrt_info * jacobian_sx;
      }

      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_sy(jacobians[4]);
        jacobian_sy.setZero();

        jacobian_sy.block<3, 1>(O_P, 0) =
            -Sophus::SO3d::exp(forward_propagated_sv).matrix() *
            (unit_y_diagonal_matrix * preintegration->linearized_linear_velocity * dtd + dp_dsy -
             preintegration->corrected_preintegration_gamma.toRotationMatrix() * unit_y_diagonal_matrix *
                 preintegration->input_linear_velocity * dtd);

        jacobian_sy = sqrt_info * jacobian_sy;
      }

      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_sw(jacobians[5]);
        jacobian_sw.setZero();

        jacobian_sw.block<3, 1>(O_P, 0) =
            -Sophus::SO3d::exp(forward_propagated_sw).matrix() *
            (dp_dsw -
             preintegration->corrected_preintegration_gamma.toRotationMatrix() *
                 Utility::skewSymmetric(Jr_dr_dsw * dq_dsw) * sv * preintegration->input_linear_velocity * dtd +
             Utility::skewSymmetric(Jr_positive_td * preintegration->linearized_angular_velocity * dtd) *
                 (forward_propagated_sv + preintegration->corrected_preintegration_alpha -
                  preintegration->corrected_preintegration_gamma * back_propagated_sv));

        jacobian_sw.block<3, 1>(O_R, 0) = -Jr_delta_theta_inv * Sophus::SO3d::exp(-raw_residual_theta).matrix() *
                                          Sophus::SO3d::exp(back_propagated_sw).matrix() *
                                          (preintegration->corrected_preintegration_gamma.inverse().toRotationMatrix() *
                                               Jr_positive_td * preintegration->linearized_angular_velocity * dtd +
                                           Jr_dr_dsw * dq_dsw);

        jacobian_sw = sqrt_info * jacobian_sw;
      }

      if (jacobians[6]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_td(jacobians[6]);
        jacobian_td.block<3, 1>(O_P, 0) =
            -Sophus::SO3d::exp(forward_propagated_sw).matrix() *
            (sv * preintegration->linearized_linear_velocity -
             preintegration->corrected_preintegration_gamma.toRotationMatrix() * sv *
                 preintegration->input_linear_velocity +
             Utility::skewSymmetric(Jr_positive_td * sw * preintegration->linearized_angular_velocity) *
                 (forward_propagated_sv + preintegration->corrected_preintegration_alpha -
                  preintegration->corrected_preintegration_gamma.toRotationMatrix() * back_propagated_sv));
        jacobian_td.block<3, 1>(O_R, 0) =
            -Jr_delta_theta_inv * Sophus::SO3d::exp(-raw_residual_theta).matrix() *
            (Sophus::SO3d::exp(back_propagated_sw).matrix() *
                 preintegration->corrected_preintegration_gamma.inverse().toRotationMatrix() * Jr_positive_td * sw *
                 preintegration->linearized_angular_velocity -
             Jr_negative_td * sw * preintegration->input_angular_velocity);

        jacobian_td = sqrt_info * jacobian_td;
      }
    }

    return true;
  }

  WheelIntegrationBase *preintegration;
};

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

#include "simpleReprojectionTwoFrameOneCamFactor.h"
#include <ceres/gradient_checker.h>
#include "pose_local_parameterization.h"

Eigen::Matrix2d SimpleReprojectionTwoFrameOneCamFactor::sqrt_info;
double SimpleReprojectionTwoFrameOneCamFactor::sum_t;

SimpleReprojectionTwoFrameOneCamFactor::SimpleReprojectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i,
                                                                               const Eigen::Vector3d &_pts_j,
                                                                               const Eigen::Quaterniond &_qic,
                                                                               const Eigen::Vector3d &_tic,
                                                                               double &_inv_dep_i)
    : pts_i(_pts_i), pts_j(_pts_j), qic(_qic), tic(_tic), inv_dep_i(_inv_dep_i){};

bool SimpleReprojectionTwoFrameOneCamFactor::Evaluate(double const *const *parameters, double *residuals,
                                                      double **jacobians) const {
  TicToc tic_toc;
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;

  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
  Eigen::Map<Eigen::Vector2d> residual(residuals);

  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

  residual = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d ric = qic.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

    reduce = sqrt_info * reduce;

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

      Eigen::Matrix<double, 3, 6> jaco_i;
      jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
      jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

      Eigen::Matrix<double, 3, 6> jaco_j;
      jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }
  }
  sum_t += tic_toc.toc();

  return true;
}

void SimpleReprojectionTwoFrameOneCamFactor::check(double **parameters) {
  double *res = new double[2];
  double **jaco = new double *[2];
  jaco[0] = new double[2 * 7];
  jaco[1] = new double[2 * 7];
  Evaluate(parameters, res, jaco);
  puts("check begins");

  puts("my");

  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl << std::endl;

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
  Eigen::Vector2d residual;

  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
  residual = sqrt_info * residual;

  puts("num");
  std::cout << residual.transpose() << std::endl;

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 20> num_jacobian;
  for (int k = 0; k < 20; k++) {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * Utility::deltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * Utility::deltaQ(delta);

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Vector2d tmp_residual;

    double dep_j = pts_camera_j.z();
    tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

    tmp_residual = sqrt_info * tmp_residual;

    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  std::cout << num_jacobian.block<2, 6>(0, 0) << std::endl;
  std::cout << num_jacobian.block<2, 6>(0, 6) << std::endl;
}

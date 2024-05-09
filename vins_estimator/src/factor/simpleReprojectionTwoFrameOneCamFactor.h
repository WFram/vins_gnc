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

#include <ceres/ceres.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class SimpleReprojectionTwoFrameOneCamFactor : public ceres::SizedCostFunction<2, 7, 7> {
 public:
  SimpleReprojectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                         const Eigen::Quaterniond &_qic, const Eigen::Vector3d &_tic,
                                         double &_inv_dep_i);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Quaterniond qic;
  Eigen::Vector3d tic;
  double inv_dep_i;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
//
// Created by wfram on 12/11/22.
//

#ifndef VINS_ALTPROJECTIONTWOFRAMETWOCAMFACTOR_H
#define VINS_ALTPROJECTIONTWOFRAMETWOCAMFACTOR_H

#pragma once

#include <ceres/ceres.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class AltProjectionTwoFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 1, 1, 1> {
 public:
  AltProjectionTwoFrameTwoCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                    const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                                    const double _td_i, const double _td_j);
  double ComputeResidual(double const *pPi, double const *pPj, double const *pTic, double const *pTic2,
                         double const *p_inv_dep_i, double const *p_td);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Vector3d velocity_i, velocity_j;
  double td_i, td_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};

#endif  // VINS_ALTPROJECTIONTWOFRAMETWOCAMFACTOR_H

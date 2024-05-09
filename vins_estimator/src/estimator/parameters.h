/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../utility/utility.h"

using namespace std;

const double FOCAL_LENGTH = 532.03125;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
// #define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;
extern int ESTIMATE_EXTRINSIC_WHEEL;
extern int ESTIMATE_INTRINSIC_WHEEL;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern double WO_LIN_VEL_XY_N;
extern double WO_LIN_VEL_Z_N;
extern double WO_ANG_VEL_XY_N;
extern double WO_ANG_VEL_Z_N;
extern double SX;
extern double SY;
extern double SW;

extern double ROLL_N, PITCH_N, ZPW_N;
extern double ROLL_N_INV, PITCH_N_INV, ZPW_N_INV;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Matrix3d RIO;
extern Eigen::Vector3d TIO;

extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string IN_CALIB_RESULT_PATH;
extern std::string INTRINSIC_ITERATE_PATH;
extern std::string EXTRINSIC_WHEEL_ITERATE_PATH;
extern std::string EXTRINSIC_CAM_ITERATE_PATH;
extern std::string PROCESS_TIME_PATH;
extern std::string TD_WHEEL_PATH;
extern std::string TD_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string GROUNDTRUTH_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern std::string WHEEL_TOPIC;
extern double TIMESHIFT_INERTIAL;
extern double OFFSET_SIM;
extern double TIMESHIFT_WHEEL;
extern int ESTIMATE_TD_INERTIAL;
extern int ESTIMATE_TD_WHEEL;
extern double TRANSLATION_FAILURE_THRESHOLD;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int RESIZE;
extern int STEREO;
extern int USE_CAMERA;
extern int USE_SEMANTICS;
extern int USE_IMU;
extern int USE_WHEEL;
extern int USE_PLANE;
extern int ONLY_INITIAL_WITH_IMU;
extern int ONLY_INITIAL_WITH_WHEEL;
extern int MULTIPLE_THREAD;
extern map<int, Eigen::Vector3d> pts_gt;

extern int PREDICT_BY_WHEELS;
extern int USE_SIMPLE_REPROJECTION_FACTORS;
extern double VO_FAILURE_WEIGHT;

extern std::string IMAGE_LEFT_RAW_TOPIC, IMAGE_RIGHT_RAW_TOPIC;
extern std::string IMAGE_LEFT_SEMANTIC_TOPIC, IMAGE_RIGHT_SEMANTIC_TOPIC;
extern std::string FEATURE0_TOPIC, FEATURE1_TOPIC;
extern std::string GROUNDTRUTH_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_RAW_TRACK;
extern int SHOW_SEMANTIC_TRACK;
extern int FLOW_BACK;

extern int DILATING_KERNEL_SIZE;
extern std::string SEMANTIC_PALETTE;

extern int USE_GNC;
extern int ADAPTIVE_GNC;
extern double FIXED_SHAPE;
extern double CONVEXITY_INIT;
extern double CONVEXITY_MARGIN;
extern double CONVEXITY_UPDATE;

extern double FEATURE_WEIGHT_THRESHOLD;
extern int SHOW_IMAGE_WEIGHT;
extern double MAX_DEPTH;

extern double REGULARIZER;

using Feature = Eigen::Matrix<double, 8, 1>;
using FeatureFrame = map<int, vector<pair<int, Feature>>>;

enum class SurrogateLossType { L1 = 0, GM, LECLERC, TLS, ADAPTIVE };

extern SurrogateLossType LOSS_TYPE;

void readParameters(const std::string& config_file);

enum SIZE_PARAMETERIZATION { SIZE_POSE = 7, SIZE_ROTATION = 4, SIZE_SPEEDBIAS = 9, SIZE_FEATURE = 1 };

enum StateOrder {
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12,
};

enum CameraExtrinsicAdjustType {
  ADJUST_CAM_TRANSLATION,
  ADJUST_CAM_ROTATION,
  ADJUST_CAM_ALL,
  ADJUST_CAM_NO_Z,
  ADJUST_CAM_NO_ROTATION_NO_Z
};
enum WheelExtrinsicAdjustType {
  ADJUST_WHEEL_TRANSLATION,
  ADJUST_WHEEL_ROTATION,
  ADJUST_WHEEL_ALL,
  ADJUST_WHEEL_NO_Z,
  ADJUST_WHEEL_NO_ROTATION_NO_Z
};
extern CameraExtrinsicAdjustType CAM_EXT_ADJ_TYPE;
extern WheelExtrinsicAdjustType WHEEL_EXT_ADJ_TYPE;
enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

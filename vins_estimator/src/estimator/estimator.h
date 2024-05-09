/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <thread>
#include <unordered_map>

#include "../factor/altProjectionTwoFrameOneCamFactor.h"
#include "../factor/altProjectionTwoFrameTwoCamFactor.h"
#include "../factor/altSimpleReprojectionTwoFrameOneCamFactor.h"
#include "../factor/altSimpleReprojectionTwoFrameTwoCamFactor.h"
#include "../factor/imu_factor.h"
#include "../factor/marginalization_factor.h"
#include "../factor/orientation_local_parameterization.h"
#include "../factor/origin_plane_factor.h"
#include "../factor/plane_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/regularization_factor.h"
#include "../factor/simpleReprojectionTwoFrameOneCamFactor.h"
#include "../factor/simpleReprojectionTwoFrameTwoCamFactor.h"
#include "../factor/wheel_factor.h"
#include "../featureTracker/feature_tracker.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../initial/initial_sfm.h"
#include "../initial/solve_5pts.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"
#include "feature_manager.h"
#include "parameters.h"
#include "surrogate_loss.h"

struct FastInertialOdometryPrediction {
  double latest_time;
  Eigen::Vector3d latest_position;
  Eigen::Vector3d latest_velocity;
  Eigen::Vector3d latest_acc_bias;
  Eigen::Vector3d latest_gyro_bias;
  Eigen::Vector3d latest_linear_acceleration;
  Eigen::Vector3d latest_angular_velocity;
  Eigen::Quaterniond latest_orientation;
};

struct FastWheelOdometryPrediction {
  double latest_time;
  Eigen::Vector3d latest_position;
  Eigen::Vector3d latest_velocity;
  double latest_sx;
  double latest_sy;
  double latest_sw;
  Eigen::Vector3d latest_linear_velocity;
  Eigen::Vector3d latest_angular_velocity;
  Eigen::Quaterniond latest_orientation;
};

class Estimator {
 public:
  Estimator();

  ~Estimator();

  void setParameter();

  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);

  void inputIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

  void inputWheel(double t, const Vector3d &linearVelocity, const Vector3d &angularVelocity);

  void inputFeature(double t, const FeatureFrame &featureFrame);

  void processInputImages(
      double t, const std::pair<const cv::Mat &, const cv::Mat &> &left_image_pair,
      const std::pair<const cv::Mat &, const cv::Mat &> &right_image_pair = std::make_pair(cv::Mat(), cv::Mat()));

  void processInertialMeasurements(double t, double dt, const Vector3d &linear_acceleration,
                                   const Vector3d &angular_velocity);

  void processWheelMeasurements(double t, double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity);

  void integrateWheelPreintegration(double t, Eigen::Vector3d &P, Eigen::Quaterniond &Q,
                                    const Eigen::Matrix<double, 7, 1> &pose);

  void processImage(const FeatureFrame &image, const double header);

  void processMeasurements();

  void changeSensorType(int use_imu, int use_stereo);

  void clearState();

  bool checkIMUObservability();

  void checkWheelYawVelocity(const Vector3d &angular_velocity);

  void checkIMUYawVelocity(const Vector3d &angular_velocity);

  bool initialStructure();

  bool alignVisualStructure();

  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

  void slideWindow();

  void slideWindowNew();

  void slideWindowOld();

  void optimization();

  double initMu();

  void optimizeVIO(bool flagOptimizeWeight);

  void vector2double();

  void double2vector();
  bool detectFailure();
  bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                      vector<pair<double, Eigen::Vector3d>> &gyrVector);
  bool getWheelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &velVector,
                        vector<pair<double, Eigen::Vector3d>> &gyrVector);
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth, Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  void fastPredictIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
  void fastPredictWheelOdometry(double t, const Eigen::Vector3d &linear_velocity,
                                const Eigen::Vector3d &angular_velocity);
  void fastPredictPureWheelOdometry(double t, const Eigen::Vector3d &linear_velocity,
                                    const Eigen::Vector3d &angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q,
                                    Eigen::Vector3d &V);
  bool imuDataAvailable(double t);
  bool wheelDataAvailable(double t);
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
  void initPlane();

  enum SolverFlag { INITIALIZATION, VIO };

  enum MarginalizationFlag { KEYFRAME = 0, NON_KEYFRAME = 1 };

  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mGTBuf;
  std::mutex mWOBuf;
  std::mutex mInertialPropagate;
  std::mutex mWheelPropagate;
  queue<pair<double, Eigen::Vector3d>> acc_data_buffer;
  queue<pair<double, Eigen::Vector3d>> wheel_odometry_linear_velocity_buffer;
  queue<pair<double, Eigen::Vector3d>> gyro_data_buffer;
  queue<pair<double, Eigen::Vector3d>> wheel_odometry_angular_velocity_buffer;
  queue<pair<double, cv::Mat>> imageBuf_0;
  queue<pair<double, cv::Mat>> imageBuf_1;
  queue<pair<double, FeatureFrame>> featureBuffer;
  queue<pair<double, Eigen::Matrix<double, 7, 1>>> groundtruthBuf;
  double previousInertialStamp, currentInertialStamp;
  double previousWheelStamp, currentWheelStamp;
  bool openExEstimation;
  bool openExWheelEstimation;
  bool openIxEstimation;
  bool openPlaneEstimation;
  std::thread trackThread;
  std::thread processThread;

  FeatureTracker featureTracker;

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Vector3d gravity_vector;

  Matrix3d ric[2];
  Quaterniond qic[2];
  Vector3d tic[2];
  Matrix3d rio;
  Vector3d tio;

  Matrix3d rpw;
  double zpw;

  double sx = 1, sy = 1, sw = 1;
  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td_inertial;
  double td_wheel;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
  WheelIntegrationBase *pre_integrations_wheel[(WINDOW_SIZE + 1)];
  Vector3d latest_linear_acceleration, latest_angular_velocity;

  Vector3d latest_wheel_linear_velocity, latest_wheel_angular_velocity;

  std::vector<double> variances;

  double fixed_shape;
  double convexity_init;
  double convexity_update;

  vector<double> dt_buffer[(WINDOW_SIZE + 1)];
  vector<Vector3d> linear_acceleration_buffer[(WINDOW_SIZE + 1)];
  vector<Vector3d> angular_velocity_buffer[(WINDOW_SIZE + 1)];

  vector<double> wheel_dt_buffer[(WINDOW_SIZE + 1)];
  vector<Vector3d> wheel_linear_velocity_buffer[(WINDOW_SIZE + 1)];
  vector<Vector3d> wheel_angular_velocity_buffer[(WINDOW_SIZE + 1)];

  int frame_count;
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
  int number_of_processed_images;

  FeatureManager f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool first_imu;
  bool first_wheel_odometry;
  bool is_valid, is_key;
  bool failure_occur;

  vector<Vector3d> point_cloud;
  vector<Vector3d> margin_cloud;
  vector<Vector3d> key_poses;
  double initial_timestamp;

  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];

  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];

  double para_Feature[NUM_OF_F][SIZE_FEATURE];

  double para_FeatureWeight[NUM_OF_F][SIZE_FEATURE];

  double para_Ex_Pose[2][SIZE_POSE];

  double para_Ex_Pose_wheel[1][SIZE_POSE];

  double para_plane_R[1][SIZE_ROTATION];
  double para_plane_Z[1][1];

  double para_Ix_sx_wheel[1][1];
  double para_Ix_sy_wheel[1][1];
  double para_Ix_sw_wheel[1][1];

  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Td_wheel[1][1];
  double para_Tr[1][1];

  int loop_window_index;

  MarginalizationInfo *last_marginalization_info;

  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> image_frame_history;
  IntegrationBase *tmp_inertial_pre_integration;
  WheelIntegrationBase *tmp_wheel_pre_integration;

  Eigen::Vector3d initP;
  Eigen::Matrix3d initR;

  FastInertialOdometryPrediction fast_inertial_odometry_prediction;
  FastWheelOdometryPrediction fast_wheel_odometry_prediction;

  bool firstInertialPoseInitialized;
  bool initThreadFlag;
};

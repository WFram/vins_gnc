/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../factor/orientation_subset_parameterization.h"
#include "../factor/pose_subset_parameterization.h"
#include "../utility/visualization.h"

Estimator::Estimator() : f_manager{Rs} {
  ROS_INFO("init begins");

  for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
    pre_integrations[i] = nullptr;
    pre_integrations_wheel[i] = nullptr;
  }
  tmp_inertial_pre_integration = nullptr;
  tmp_wheel_pre_integration = nullptr;
  last_marginalization_info = nullptr;

  initThreadFlag = false;
  clearState();

  variances.clear();
}

Estimator::~Estimator() {
  if (MULTIPLE_THREAD) {
    processThread.join();
    printf("join thread \n");
  }
}

void Estimator::clearState() {
  mProcess.lock();
  while (!acc_data_buffer.empty()) acc_data_buffer.pop();
  while (!gyro_data_buffer.empty()) gyro_data_buffer.pop();
  while (!featureBuffer.empty()) featureBuffer.pop();

  while (!wheel_odometry_linear_velocity_buffer.empty()) wheel_odometry_linear_velocity_buffer.pop();
  while (!wheel_odometry_angular_velocity_buffer.empty()) wheel_odometry_angular_velocity_buffer.pop();

  while (!imageBuf_0.empty()) imageBuf_0.pop();
  while (!imageBuf_1.empty()) imageBuf_1.pop();

  previousInertialStamp = -1;
  previousWheelStamp = -1;
  currentInertialStamp = 0;
  currentWheelStamp = 0;
  openExEstimation = 0;
  openPlaneEstimation = 0;
  openExWheelEstimation = 0;
  openIxEstimation = 0;
  initP = Eigen::Vector3d(0, 0, 0);
  initR = Eigen::Matrix3d::Identity();
  number_of_processed_images = 0;
  firstInertialPoseInitialized = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buffer[i].clear();
    linear_acceleration_buffer[i].clear();
    angular_velocity_buffer[i].clear();

    if (pre_integrations[i] != nullptr) {
      delete pre_integrations[i];
    }
    pre_integrations[i] = nullptr;

    wheel_dt_buffer[i].clear();
    wheel_linear_velocity_buffer[i].clear();
    wheel_angular_velocity_buffer[i].clear();
    if (pre_integrations_wheel[i] != nullptr) {
      delete pre_integrations_wheel[i];
    }
    pre_integrations_wheel[i] = nullptr;
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Vector3d::Zero();
    ric[i] = Matrix3d::Identity();
    qic[i] = ric[i];
  }
  tio = Vector3d::Zero();
  rio = Matrix3d::Identity();
  sx = 0;
  sy = 0;
  sw = 0;

  rpw = Matrix3d::Identity();
  zpw = 0;

  first_imu = false, first_wheel_odometry = false, sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;
  solver_flag = INITIALIZATION;
  initial_timestamp = 0;

  for (auto &it : image_frame_history) {
    if (it.second.imu_preintegration != NULL) {
      delete it.second.imu_preintegration;
      it.second.imu_preintegration = NULL;
    }
    if (it.second.pre_integration_wheel != NULL) {
      delete it.second.pre_integration_wheel;
      it.second.pre_integration_wheel = NULL;
    }
  }

  image_frame_history.clear();

  if (tmp_inertial_pre_integration != nullptr) delete tmp_inertial_pre_integration;
  if (tmp_wheel_pre_integration != nullptr) delete tmp_wheel_pre_integration;
  if (last_marginalization_info != nullptr) delete last_marginalization_info;

  tmp_inertial_pre_integration = nullptr;
  tmp_wheel_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  f_manager.clearState();

  failure_occur = 0;

  mProcess.unlock();
}

void Estimator::setParameter() {
  mProcess.lock();
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    qic[i] = ric[i];
    cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }
  tio = TIO;
  rio = RIO;
  cout << " exitrinsic wheel " << endl << rio << endl << tio.transpose() << endl;
  sx = SX;
  sy = SY;
  sw = SW;
  cout << " initrinsic wheel " << endl << sx << " " << sy << " " << sw << endl;

  f_manager.setRic(ric);
  ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

  SimpleReprojectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  SimpleReprojectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

  AltProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  AltProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

  AltSimpleReprojectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  AltSimpleReprojectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

  fixed_shape = FIXED_SHAPE;
  convexity_init = CONVEXITY_INIT;
  convexity_update = CONVEXITY_UPDATE;

  td_inertial = TIMESHIFT_INERTIAL;
  td_wheel = TIMESHIFT_WHEEL;
  gravity_vector = G;
  cout << "set g " << gravity_vector.transpose() << endl;
  featureTracker.readIntrinsicParameter(CAM_NAMES);

  std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
  if (MULTIPLE_THREAD && !initThreadFlag) {
    initThreadFlag = true;
    processThread = std::thread(&Estimator::processMeasurements, this);
  }
  mProcess.unlock();

  SemanticPalette semantic_palette;
  if (SEMANTIC_PALETTE == "ade20k")
    semantic_palette = SemanticPalette::ADE20K;
  else if (SEMANTIC_PALETTE == "cityscapes")
    semantic_palette = SemanticPalette::CITYSCAPES;
  else if (SEMANTIC_PALETTE == "pascal_voc")
    semantic_palette = SemanticPalette::PASCAL_VOC;

  featureTracker.initializeSemanticMap(semantic_palette);
}

void Estimator::changeSensorType(int use_imu, int use_stereo) {
  bool restart = false;
  mProcess.lock();
  if (!use_imu && !use_stereo)
    printf("at least use two sensors! \n");
  else {
    if (USE_IMU != use_imu) {
      USE_IMU = use_imu;
      if (USE_IMU) {
        restart = true;
      } else {
        if (last_marginalization_info != nullptr) delete last_marginalization_info;

        tmp_inertial_pre_integration = nullptr;
        last_marginalization_info = nullptr;
        last_marginalization_parameter_blocks.clear();
      }
    }

    STEREO = use_stereo;
    printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
  }
  mProcess.unlock();
  if (restart) {
    clearState();
    setParameter();
  }
}

void Estimator::processInputImages(double t, const std::pair<const cv::Mat &, const cv::Mat &> &left_image_pair,
                                   const std::pair<const cv::Mat &, const cv::Mat &> &right_image_pair) {
  number_of_processed_images++;
  FeatureFrame feature_frame;
  TicToc feature_tracker_inference_time;

  const auto &left_raw_image = left_image_pair.first;

  if (right_image_pair.first.empty())
    feature_frame = featureTracker.trackImage(t, left_image_pair);
  else
    feature_frame = featureTracker.trackImage(t, left_image_pair, right_image_pair);

  if (SHOW_RAW_TRACK) {
    cv::Mat tracked_raw_image = featureTracker.getRawTrackImage();
    publishRawTrackImage(tracked_raw_image, t);
  }

  if (SHOW_SEMANTIC_TRACK && USE_SEMANTICS) {
    cv::Mat tracked_semantic_image = featureTracker.getSemanticTrackImage();
    publishSemanticTrackImage(tracked_semantic_image, t);
  }

  if (MULTIPLE_THREAD) {
    if (number_of_processed_images % 2 == 0) {
      mBuf.lock();
      featureBuffer.push(make_pair(t, feature_frame));
      imageBuf_0.push(make_pair(t, left_raw_image.clone()));
      mBuf.unlock();
    }
  } else {
    mBuf.lock();
    featureBuffer.push(make_pair(t, feature_frame));
    imageBuf_0.push(make_pair(t, left_raw_image.clone()));
    mBuf.unlock();
    TicToc measurement_processing_time;
    processMeasurements();
    printf("Measurement processing time: %f\n", measurement_processing_time.toc());
  }
}

void Estimator::inputIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {
  mBuf.lock();
  acc_data_buffer.push(make_pair(t, linear_acceleration));
  gyro_data_buffer.push(make_pair(t, angular_velocity));
  mBuf.unlock();

  if (solver_flag == VIO) {
    mInertialPropagate.lock();
    fastPredictIMU(t, linear_acceleration, angular_velocity);
    pubLatestInertialOdometry(fast_inertial_odometry_prediction.latest_position,
                              fast_inertial_odometry_prediction.latest_orientation,
                              fast_inertial_odometry_prediction.latest_velocity, t);
    mInertialPropagate.unlock();
  }
}

void Estimator::inputWheel(double t, const Vector3d &linear_velocity, const Vector3d &angular_velocity) {
  mWOBuf.lock();
  wheel_odometry_linear_velocity_buffer.push(make_pair(t, linear_velocity));
  wheel_odometry_angular_velocity_buffer.push(make_pair(t, angular_velocity));
  mWOBuf.unlock();

  if (solver_flag == VIO) {
    mWheelPropagate.lock();
    fastPredictWheelOdometry(t, linear_velocity, angular_velocity);
    pubLatestWheelOdometry(fast_wheel_odometry_prediction.latest_position,
                           fast_wheel_odometry_prediction.latest_orientation,
                           fast_wheel_odometry_prediction.latest_velocity, t);
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    fastPredictPureWheelOdometry(t, linear_velocity, angular_velocity, position, orientation, velocity);
    pubPureWheelLatestOdometry(position, orientation, velocity, t);
    mWheelPropagate.unlock();
  }
}

void Estimator::inputFeature(double t, const FeatureFrame &featureFrame) {
  mBuf.lock();
  featureBuffer.push(make_pair(t, featureFrame));
  mBuf.unlock();

  if (!MULTIPLE_THREAD) processMeasurements();
}

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &linear_acceleration_data,
                               vector<pair<double, Eigen::Vector3d>> &angular_velocity_data) {
  if (acc_data_buffer.empty()) {
    printf("IMU data is not received\n");
    return false;
  }
  if (t1 <= acc_data_buffer.back().first) {
    while (acc_data_buffer.front().first <= t0) {
      acc_data_buffer.pop();
      gyro_data_buffer.pop();
    }
    while (acc_data_buffer.front().first < t1) {
      linear_acceleration_data.push_back(acc_data_buffer.front());
      acc_data_buffer.pop();
      angular_velocity_data.push_back(gyro_data_buffer.front());
      gyro_data_buffer.pop();
    }
    linear_acceleration_data.push_back(acc_data_buffer.front());
    angular_velocity_data.push_back(gyro_data_buffer.front());
  } else {
    printf("Waiting for IMU readings\n");
    return false;
  }
  return true;
}

bool Estimator::getWheelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &linear_velocity_data,
                                 vector<pair<double, Eigen::Vector3d>> &angular_velocity_data) {
  if (wheel_odometry_linear_velocity_buffer.empty()) {
    printf("Wheel odometry is not received\n");
    return false;
  }
  if (t1 <= wheel_odometry_linear_velocity_buffer.back().first) {
    while (wheel_odometry_linear_velocity_buffer.front().first <= t0) {
      wheel_odometry_linear_velocity_buffer.pop();
      wheel_odometry_angular_velocity_buffer.pop();
    }
    while (wheel_odometry_linear_velocity_buffer.front().first < t1) {
      linear_velocity_data.push_back(wheel_odometry_linear_velocity_buffer.front());
      wheel_odometry_linear_velocity_buffer.pop();
      angular_velocity_data.push_back(wheel_odometry_angular_velocity_buffer.front());
      wheel_odometry_angular_velocity_buffer.pop();
    }
    linear_velocity_data.push_back(wheel_odometry_linear_velocity_buffer.front());
    angular_velocity_data.push_back(wheel_odometry_angular_velocity_buffer.front());
  } else {
    printf("Waiting for wheel odometry readings\n");
    return false;
  }
  return true;
}

bool Estimator::imuDataAvailable(double t) {
  if (!acc_data_buffer.empty() && t <= acc_data_buffer.back().first)
    return true;
  else
    return false;
}
bool Estimator::wheelDataAvailable(double t) {
  if (!wheel_odometry_linear_velocity_buffer.empty() && t <= wheel_odometry_linear_velocity_buffer.back().first)
    return true;
  else
    return false;
}

void Estimator::processMeasurements() {
  while (true) {
    pair<double, FeatureFrame> feature;
    vector<pair<double, Eigen::Vector3d>> inertial_linear_acceleration_data, inertial_angular_velocity_data;
    vector<pair<double, Eigen::Vector3d>> wheel_linear_velocity_data, wheel_angular_velocity_data;

    if (!featureBuffer.empty()) {
      feature = featureBuffer.front();
      currentInertialStamp = feature.first + td_inertial;
      currentWheelStamp = currentInertialStamp - td_wheel;
      while (true) {
        if ((!USE_IMU || imuDataAvailable(feature.first + td_inertial)))
          break;
        else {
          printf("Waiting for IMU data ... \n");
          if (!MULTIPLE_THREAD) return;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
      while (true) {
        if ((!USE_WHEEL || wheelDataAvailable(feature.first + td_inertial - td_wheel)))
          break;
        else {
          if (!MULTIPLE_THREAD) return;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
      mBuf.lock();
      if (USE_IMU) {
        getIMUInterval(previousInertialStamp, currentInertialStamp, inertial_linear_acceleration_data,
                       inertial_angular_velocity_data);
      }
      featureBuffer.pop();
      mBuf.unlock();
      mWOBuf.lock();
      if (USE_WHEEL) {
        getWheelInterval(previousWheelStamp, currentWheelStamp, wheel_linear_velocity_data,
                         wheel_angular_velocity_data);
      }
      mWOBuf.unlock();

      if (USE_IMU) {
        if (!firstInertialPoseInitialized) initFirstIMUPose(inertial_linear_acceleration_data);
        for (size_t i = 0; i < inertial_linear_acceleration_data.size(); i++) {
          double dt;
          if (i == 0)
            dt = inertial_linear_acceleration_data[i].first - previousInertialStamp;
          else if (i == inertial_linear_acceleration_data.size() - 1)
            dt = currentInertialStamp - inertial_linear_acceleration_data[i - 1].first;
          else
            dt = inertial_linear_acceleration_data[i].first - inertial_linear_acceleration_data[i - 1].first;
          processInertialMeasurements(inertial_linear_acceleration_data[i].first, dt,
                                      inertial_linear_acceleration_data[i].second,
                                      inertial_angular_velocity_data[i].second);
        }
      }
      if (USE_WHEEL) {
        for (size_t i = 0; i < wheel_linear_velocity_data.size(); i++) {
          double dt;
          if (i == 0)
            dt = wheel_linear_velocity_data[i].first - previousWheelStamp;
          else if (i == wheel_linear_velocity_data.size() - 1)
            dt = currentWheelStamp - wheel_linear_velocity_data[i - 1].first;
          else
            dt = wheel_linear_velocity_data[i].first - wheel_linear_velocity_data[i - 1].first;
          processWheelMeasurements(wheel_linear_velocity_data[i].first, dt, wheel_linear_velocity_data[i].second,
                                   wheel_angular_velocity_data[i].second);
        }
      }
      mProcess.lock();
      processImage(feature.second, feature.first);
      previousInertialStamp = currentInertialStamp;
      previousWheelStamp = currentWheelStamp;

      printStatistics(*this, 0);

      std_msgs::Header header;
      header.frame_id = "world";
      header.stamp = ros::Time(feature.first);

      pubOdometry(*this, header);
      Eigen::Matrix<double, 7, 1> pose;
      pubGroundTruth(*this, header, pose, td_inertial);
      pubKeyPoses(*this, header);
      pubCameraPose(*this, header);
      pubPointCloud(*this, header);
      pubKeyframe(*this);
      pubTF(*this, header);
      mProcess.unlock();
    }

    if (!MULTIPLE_THREAD) break;

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &inertial_linear_acceleration_data) {
  printf("Initialize first IMU pose\n");
  firstInertialPoseInitialized = true;
  Eigen::Vector3d average_acceleration(0, 0, 0);
  int number_of_measurements = static_cast<int>(inertial_linear_acceleration_data.size());
  for (size_t i = 0; i < inertial_linear_acceleration_data.size(); i++) {
    average_acceleration = average_acceleration + inertial_linear_acceleration_data[i].second;
  }
  average_acceleration = average_acceleration / number_of_measurements;
  printf("Average acceleration %f %f %f\n", average_acceleration.x(), average_acceleration.y(),
         average_acceleration.z());
  Matrix3d orientation = Utility::g2R(average_acceleration);
  double yaw = Utility::R2ypr(orientation).x();
  orientation = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * orientation;
  Rs[0] = orientation;
  cout << "Initial orientation from IMU: " << endl << Rs[0] << endl;
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
  Ps[0] = p;
  Rs[0] = r;
  initP = p;
  initR = r;
}

void Estimator::processInertialMeasurements(double t, double dt, const Vector3d &linear_acceleration,
                                            const Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    latest_linear_acceleration = linear_acceleration;
    latest_angular_velocity = angular_velocity;
  }

  if (!pre_integrations[frame_count]) {
    pre_integrations[frame_count] =
        new IntegrationBase{latest_linear_acceleration, latest_angular_velocity, Bas[frame_count], Bgs[frame_count]};
  }
  if (frame_count != 0) {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
    tmp_inertial_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buffer[frame_count].push_back(dt);
    linear_acceleration_buffer[frame_count].push_back(linear_acceleration);
    angular_velocity_buffer[frame_count].push_back(angular_velocity);

    int j = frame_count;
    Vector3d left_linear_acceleration = Rs[j] * (latest_linear_acceleration - Bas[j]) - gravity_vector;
    Vector3d mid_angular_velocity = 0.5 * (latest_angular_velocity + angular_velocity) - Bgs[j];
    Rs[j] *= Utility::deltaQ(mid_angular_velocity * dt).toRotationMatrix();
    Vector3d right_linear_acceleration = Rs[j] * (linear_acceleration - Bas[j]) - gravity_vector;
    Vector3d mid_linear_acceleration = 0.5 * (left_linear_acceleration + right_linear_acceleration);
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * mid_linear_acceleration;
    Vs[j] += dt * mid_linear_acceleration;
  }
  latest_linear_acceleration = linear_acceleration;
  latest_angular_velocity = angular_velocity;
}

void Estimator::processWheelMeasurements(double t, double dt, const Vector3d &linear_velocity,
                                         const Vector3d &angular_velocity) {
  if (!first_wheel_odometry) {
    first_wheel_odometry = true;
    latest_wheel_linear_velocity = linear_velocity;
    latest_wheel_angular_velocity = angular_velocity;
  }

  if (!pre_integrations_wheel[frame_count]) {
    pre_integrations_wheel[frame_count] =
        new WheelIntegrationBase{latest_wheel_linear_velocity, latest_wheel_angular_velocity, sx, sy, sw, td_wheel};
  }
  if (frame_count != 0) {
    pre_integrations_wheel[frame_count]->push_back(dt, linear_velocity, angular_velocity);

    tmp_wheel_pre_integration->push_back(dt, linear_velocity, angular_velocity);

    wheel_dt_buffer[frame_count].push_back(dt);
    wheel_linear_velocity_buffer[frame_count].push_back(linear_velocity);
    wheel_angular_velocity_buffer[frame_count].push_back(angular_velocity);

    if (PREDICT_BY_WHEELS && !USE_IMU) {
      int j = frame_count;
      Eigen::Matrix3d sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
      Vector3d left_linear_velocity = Rs[j] * sv * latest_wheel_linear_velocity;
      Vector3d mid_angular_velocity = 0.5 * sw * (latest_wheel_angular_velocity + angular_velocity);
      Eigen::Quaterniond orientation = Quaterniond(1, mid_angular_velocity(0) * dt / 2,
                                                   mid_angular_velocity(1) * dt / 2, mid_angular_velocity(2) * dt / 2);
      Rs[j] *= orientation.toRotationMatrix();
      Vector3d right_linear_velocity = Rs[j] * sv * linear_velocity;
      Vector3d mid_linear_velocity = 0.5 * (left_linear_velocity + right_linear_velocity);
      Ps[j] += mid_linear_velocity * dt;
    }
  }
  latest_wheel_linear_velocity = linear_velocity;
  latest_wheel_angular_velocity = angular_velocity;
}

void Estimator::checkWheelYawVelocity(const Vector3d &angular_velocity) {
  if (fabs(angular_velocity[2]) > 0.25) {
    ROS_WARN("Detected large yaw velocity! Turning visual constraints off");
    USE_CAMERA = 0;
  } else
    USE_CAMERA = 1;
}

void Estimator::checkIMUYawVelocity(const Vector3d &angular_velocity) {
  if (fabs(angular_velocity[2]) > 0.25) {
    ROS_WARN("Detected large yaw velocity! Set weights to 1.0");
    VectorXd wei = f_manager.getWeightVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++) wei(i) = 1.0;
    f_manager.setWeight(wei);
  }
}

void Estimator::integrateWheelPreintegration(double t, Eigen::Vector3d &P, Eigen::Quaterniond &Q,
                                             const Eigen::Matrix<double, 7, 1> &pose) {
  static bool firstPreint = false;
  static Eigen::Vector3d Pwo;
  static Eigen::Quaterniond Qwo;
  if (!firstPreint) {
    Pwo = Eigen::Vector3d(pose[4], pose[5], pose[6]);
    Qwo.w() = pose[0];
    Qwo.x() = pose[1];
    Qwo.y() = pose[2];
    Qwo.z() = pose[3];
    std::cout << "integrateWheelPreintegration initial pose: \n"
              << Pwo.transpose() << std::endl
              << Qwo.coeffs().transpose() << std::endl;
    firstPreint = true;
  } else {
    Pwo = Qwo * image_frame_history[t].pre_integration_wheel->preintegration_alpha + Pwo.eval();
    Qwo = Qwo * image_frame_history[t].pre_integration_wheel->preintegration_gamma;
  }
  P = Pwo;
  Q = Qwo;
}

void Estimator::processImage(const FeatureFrame &image, const double header) {
  ROS_DEBUG("Processing new image");
  ROS_DEBUG("Adding feature points %lu", image.size());
  if (f_manager.addFeatureCheckParallax(frame_count, image, td_inertial)) {
    marginalization_flag = KEYFRAME;
  } else {
    marginalization_flag = NON_KEYFRAME;
  }

  ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
  ROS_DEBUG("Solving for %d", frame_count);
  ROS_DEBUG("Number of features: %d", f_manager.getFeatureCount());
  Headers[frame_count] = header;

  ImageFrame image_frame(image, header);
  image_frame.imu_preintegration = tmp_inertial_pre_integration;
  image_frame.pre_integration_wheel = tmp_wheel_pre_integration;
  image_frame_history.insert(make_pair(header, image_frame));
  tmp_inertial_pre_integration =
      new IntegrationBase{latest_linear_acceleration, latest_angular_velocity, Bas[frame_count], Bgs[frame_count]};
  tmp_wheel_pre_integration =
      new WheelIntegrationBase{latest_wheel_linear_velocity, latest_wheel_angular_velocity, sx, sy, sw, td_wheel};

  if (ESTIMATE_EXTRINSIC == 2) {
    ROS_INFO("Calibrating extrinsic parameters, rotation movement is needed");
    if (frame_count != 0) {
      vector<pair<Vector3d, Vector3d>> correspondences = f_manager.getCorrespondences(frame_count - 1, frame_count);
      Matrix3d rotation_extrinsic;
      if (initial_ex_rotation.CalibrationExRotation(
              correspondences, pre_integrations[frame_count]->preintegration_gamma, rotation_extrinsic)) {
        ROS_WARN("Initial extrinsic rotation calibration success");
        ROS_WARN_STREAM("Initial extrinsic rotation: " << endl << rotation_extrinsic);
        ric[0] = rotation_extrinsic;
        qic[0] = ric[0];
        RIC[0] = rotation_extrinsic;
        ESTIMATE_EXTRINSIC = 1;
      }
    }
  }

  if (solver_flag == INITIALIZATION) {
    ROS_DEBUG("Preparing for initialization");
    if (!STEREO && USE_IMU) {
      if (frame_count == WINDOW_SIZE) {
        bool initialization_success = false;
        if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) {
          initialization_success = initialStructure();
          initial_timestamp = header;
        }
        if (initialization_success) {
          optimization();
          updateLatestStates();
          solver_flag = VIO;
          slideWindow();
          ROS_DEBUG("Initialization finished!");
        } else
          slideWindow();
      }
    }

    if (!STEREO && !USE_IMU) {
      if (frame_count == WINDOW_SIZE) {
        bool initialization_success = false;
        if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) {
          initialization_success = initialStructure();
          initial_timestamp = header;
        }
        if (initialization_success) {
          ROS_DEBUG("Solved initial structure. Going to optimization");
          optimization();
          updateLatestStates();
          solver_flag = VIO;
          slideWindow();
          ROS_DEBUG("Initialization finish!");
        } else
          slideWindow();
      }
    }

    if (STEREO && USE_IMU) {
      f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

      f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
      if (frame_count == WINDOW_SIZE) {
        map<double, ImageFrame>::iterator frame_it;
        int i = 0;
        for (frame_it = image_frame_history.begin(); frame_it != image_frame_history.end(); frame_it++) {
          frame_it->second.rotation = Rs[i];
          frame_it->second.translation = Ps[i];
          i++;
        }
        solveGyroscopeBias(image_frame_history, Bgs);

        for (int i = 0; i <= WINDOW_SIZE; i++) {
          pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
        }

        optimization();
        ROS_INFO("Optimization finished. Updating latest states");
        updateLatestStates();
        solver_flag = VIO;
        ROS_INFO("State updated. Sliding window");
        slideWindow();
        ROS_INFO("Initialization finish!");
      }
    }

    if (STEREO && !USE_IMU) {
      if (!PREDICT_BY_WHEELS) f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
      f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
      bool use_optimization = false;
      if (use_optimization) optimization();

      if (frame_count == WINDOW_SIZE) {
        if (use_optimization) optimization();
        updateLatestStates();
        solver_flag = VIO;
        slideWindow();
        ROS_DEBUG("Initialization finish!");
      }
    }

    if (frame_count < WINDOW_SIZE) {
      ROS_INFO("Accumulating frames in window. Update states. ");
      frame_count++;
      int previous_frame = frame_count - 1;
      Ps[frame_count] = Ps[previous_frame];
      Vs[frame_count] = Vs[previous_frame];
      Rs[frame_count] = Rs[previous_frame];
      Bas[frame_count] = Bas[previous_frame];
      Bgs[frame_count] = Bgs[previous_frame];
    }

    if (solver_flag == VIO && USE_PLANE) initPlane();
  } else {
    TicToc optimization_inference_time;

    if (!PREDICT_BY_WHEELS && !USE_IMU && !failure_occur) f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

    if (!failure_occur) f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    // TODO: consider if we need the optimization in failure case or not
    //    if (!detectFailure()) optimization();
    detectFailure();
    optimization();
    USE_CAMERA = 1;
    set<int> point_ids_to_remove;
    outliersRejection(point_ids_to_remove);
    if (FEATURE0_TOPIC.empty()) {
      f_manager.removeOutlier(point_ids_to_remove);
    }
    if (!MULTIPLE_THREAD) {
      if (FEATURE0_TOPIC.empty()) {
        featureTracker.removeOutliers(point_ids_to_remove);
      }
      predictPtsInNextFrame();
    }

    ROS_DEBUG("Solver costs: %fms", optimization_inference_time.toc());

    slideWindow();
    f_manager.removeFailures();
    key_poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++) key_poses.push_back(Ps[i]);

    last_R = Rs[WINDOW_SIZE];
    last_P = Ps[WINDOW_SIZE];
    last_R0 = Rs[0];
    last_P0 = Ps[0];
    updateLatestStates();
  }
}

void Estimator::initPlane() {
  ROS_DEBUG("initialize plane");
  rpw = Utility::ypr2R(Eigen::Vector3d{0, 0, 0});
  zpw = 0;

  std::cout << "Init plane:  rpw: " << Eigen::AngleAxisd(rpw).axis().transpose() << " zpw: " << zpw << std::endl;
}

bool Estimator::checkIMUObservability() {
  map<double, ImageFrame>::iterator frame_it;
  Vector3d sum_g;
  ROS_INFO("Check IMU observability");
  for (frame_it = image_frame_history.begin(), frame_it++; frame_it != image_frame_history.end(); frame_it++) {
    double dt = frame_it->second.imu_preintegration->sum_dt;
    Vector3d tmp_g = frame_it->second.imu_preintegration->preintegration_beta / dt;
    sum_g += tmp_g;
  }

  Vector3d aver_g;
  aver_g = sum_g * 1.0 / ((int)image_frame_history.size() - 1);
  double var = 0;

  for (frame_it = image_frame_history.begin(), frame_it++; frame_it != image_frame_history.end(); frame_it++) {
    double dt = frame_it->second.imu_preintegration->sum_dt;
    Vector3d tmp_g = frame_it->second.imu_preintegration->preintegration_beta / dt;
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
  }

  var = sqrt(var / ((int)image_frame_history.size() - 1));
  ROS_INFO("IMU variation %f", var);
  if (var < 0.25) {
    ROS_WARN("IMU excitation is not enough!");
    return false;
  }
  return true;
}

bool Estimator::initialStructure() {
  TicToc t_sfm;
  {
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    ROS_DEBUG("Check IMU observability");
    for (frame_it = image_frame_history.begin(), frame_it++; frame_it != image_frame_history.end(); frame_it++) {
      double dt = frame_it->second.imu_preintegration->sum_dt;
      Vector3d tmp_g = frame_it->second.imu_preintegration->preintegration_beta / dt;
      sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)image_frame_history.size() - 1);
    double var = 0;
    for (frame_it = image_frame_history.begin(), frame_it++; frame_it != image_frame_history.end(); frame_it++) {
      double dt = frame_it->second.imu_preintegration->sum_dt;
      Vector3d tmp_g = frame_it->second.imu_preintegration->preintegration_beta / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }
    var = sqrt(var / ((int)image_frame_history.size() - 1));
    if (var < 0.25) {
      ROS_WARN("IMU excitation not enough!");
    }
  }
  Quaterniond Q[frame_count + 1];
  Vector3d T[frame_count + 1];
  map<int, Vector3d> sfm_tracked_points;
  vector<SFMFeature> sfm_f;
  ROS_DEBUG("Prepare to solve SfM");
  for (auto &it_per_id : f_manager.feature) {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }
  Matrix3d relative_R;
  Vector3d relative_T;
  int l;
  if (!relativePose(relative_R, relative_T, l)) {
    ROS_WARN("Not enough features or parallax; Move device around");
    return false;
  }
  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points)) {
    ROS_WARN("global SFM failed!");
    marginalization_flag = KEYFRAME;
    return false;
  }

  map<double, ImageFrame>::iterator frame_it;
  map<int, Vector3d>::iterator it;
  frame_it = image_frame_history.begin();
  for (int i = 0; frame_it != image_frame_history.end(); frame_it++) {
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i]) {
      frame_it->second.is_key_frame = true;
      frame_it->second.rotation = Q[i].toRotationMatrix() * RIC[0].transpose();
      frame_it->second.translation = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i]) {
      i++;
    }
    Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6) {
      cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      ROS_WARN("Not enough points for solve pnp !");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      ROS_WARN("solve pnp fail!");
      return false;
    }
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.rotation = R_pnp * RIC[0].transpose();
    frame_it->second.translation = T_pnp;
  }
  if (USE_IMU) {
    if (alignVisualStructure())
      return true;
    else {
      ROS_WARN("Visual structure alignment failed");
      return false;
    }
  } else
    return true;
}

bool Estimator::alignVisualStructure() {
  ROS_DEBUG("Start visual structure alignment");
  VectorXd state;
  bool result = visualAlignment(image_frame_history, Bgs, gravity_vector, state);
  if (!result) {
    ROS_WARN("Failed to solve gravity vector");
    return false;
  }

  ROS_DEBUG("Changing state variables");
  for (int i = 0; i <= frame_count; i++) {
    Matrix3d frame_rotation = image_frame_history[Headers[i]].rotation;
    Vector3d frame_translation = image_frame_history[Headers[i]].translation;
    Ps[i] = frame_translation;
    Rs[i] = frame_rotation;
    image_frame_history[Headers[i]].is_key_frame = true;
  }

  double scale = (state.tail<1>())(0);

  if (USE_IMU) {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
      pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
  }

  ROS_DEBUG("Scaling the pose");
  for (int i = frame_count; i >= 0; i--) {
    Ps[i] = scale * Ps[i] - Rs[i] * TIC[0] - (scale * Ps[0] - Rs[0] * TIC[0]);
  }
  int keyframe_id = -1;
  map<double, ImageFrame>::iterator frame_i;
  ROS_DEBUG("Updating velocities");
  for (frame_i = image_frame_history.begin(); frame_i != image_frame_history.end(); frame_i++) {
    if (frame_i->second.is_key_frame) {
      keyframe_id++;
      Vs[keyframe_id] = frame_i->second.rotation * state.segment<3>(keyframe_id * 3);
    }
  }
  ROS_INFO_STREAM("Initial gravity vector estimate: " << gravity_vector.transpose());
  Matrix3d gravity_rotation = Utility::g2R(gravity_vector);
  double yaw = Utility::R2ypr(gravity_rotation * Rs[0]).x();
  gravity_rotation = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * gravity_rotation;
  gravity_vector = gravity_rotation * gravity_vector;
  for (int i = 0; i <= frame_count; i++) {
    Ps[i] = gravity_rotation * Ps[i];
    Rs[i] = gravity_rotation * Rs[i];
    Vs[i] = gravity_rotation * Vs[i];
  }
  ROS_INFO_STREAM("Final gravity vector estimate: " << gravity_vector.transpose());
  ROS_INFO_STREAM("First rotation: " << Utility::R2ypr(Rs[0]).transpose());

  f_manager.clearDepth();
  f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

  return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
  for (int i = 0; i < WINDOW_SIZE; i++) {
    vector<pair<Vector3d, Vector3d>> corres;
    corres = f_manager.getCorrespondences(i, WINDOW_SIZE);
    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++) {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      double threshold = FEATURE0_TOPIC.empty() ? 30 : 8;
      if (average_parallax * 460 > threshold && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
        l = i;
        ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure",
                  average_parallax * 460, l);
        return true;
      }
    }
  }
  return false;
}

void Estimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_Pose[i][0] = Ps[i].x();
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    if (USE_IMU) {
      para_SpeedBias[i][0] = Vs[i].x();
      para_SpeedBias[i][1] = Vs[i].y();
      para_SpeedBias[i][2] = Vs[i].z();

      para_SpeedBias[i][3] = Bas[i].x();
      para_SpeedBias[i][4] = Bas[i].y();
      para_SpeedBias[i][5] = Bas[i].z();

      para_SpeedBias[i][6] = Bgs[i].x();
      para_SpeedBias[i][7] = Bgs[i].y();
      para_SpeedBias[i][8] = Bgs[i].z();
    }
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();

    qic[i] = ric[i];
    para_Ex_Pose[i][3] = qic[i].x();
    para_Ex_Pose[i][4] = qic[i].y();
    para_Ex_Pose[i][5] = qic[i].z();
    para_Ex_Pose[i][6] = qic[i].w();
  }

  para_Ex_Pose_wheel[0][0] = tio.x();
  para_Ex_Pose_wheel[0][1] = tio.y();
  para_Ex_Pose_wheel[0][2] = tio.z();
  Quaterniond q{rio};
  para_Ex_Pose_wheel[0][3] = q.x();
  para_Ex_Pose_wheel[0][4] = q.y();
  para_Ex_Pose_wheel[0][5] = q.z();
  para_Ex_Pose_wheel[0][6] = q.w();

  para_Ix_sx_wheel[0][0] = sx;
  para_Ix_sy_wheel[0][0] = sy;
  para_Ix_sw_wheel[0][0] = sw;

  Quaterniond q2{rpw};
  para_plane_R[0][0] = q.x();
  para_plane_R[0][1] = q.y();
  para_plane_R[0][2] = q.z();
  para_plane_R[0][3] = q.w();
  para_plane_Z[0][0] = zpw;

  int featureSel = 0;
  int featureDel = 0;
  VectorXd dep = f_manager.getDepthVector();
  VectorXd weight = f_manager.getWeightVector();
  for (int i = 0; i < f_manager.getFeatureCount(); i++) {
    para_Feature[i][0] = dep(i);
    para_FeatureWeight[i][0] = weight(i);
  }

  para_Td[0][0] = td_inertial;
  para_Td_wheel[0][0] = td_wheel;
}

void Estimator::double2vector() {
  Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Vector3d origin_P0 = Ps[0];

  if (USE_IMU) {
    Vector3d origin_R00 = Utility::R2ypr(
        Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
      ROS_DEBUG("euler singular point!");
      rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5])
                             .toRotationMatrix()
                             .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
      Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                             .normalized()
                             .toRotationMatrix();

      Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1],
                                  para_Pose[i][2] - para_Pose[0][2]) +
              origin_P0;

      Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2]);

      Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5]);

      Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);
    }
  } else {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
      Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                  .normalized()
                  .toRotationMatrix();

      Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
    }
  }

  if (USE_IMU) {
    for (int i = 0; i < NUM_OF_CAM; i++) {
      tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
      qic[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4], para_Ex_Pose[i][5]).normalized();
      ric[i] = qic[i].toRotationMatrix();
    }
  }

  if (USE_WHEEL) {
    tio = Vector3d(para_Ex_Pose_wheel[0][0], para_Ex_Pose_wheel[0][1], para_Ex_Pose_wheel[0][2]);
    rio = Quaterniond(para_Ex_Pose_wheel[0][6], para_Ex_Pose_wheel[0][3], para_Ex_Pose_wheel[0][4],
                      para_Ex_Pose_wheel[0][5])
              .normalized()
              .toRotationMatrix();
    sx = para_Ix_sx_wheel[0][0];
    sy = para_Ix_sy_wheel[0][0];
    sw = para_Ix_sw_wheel[0][0];

    td_wheel = para_Td_wheel[0][0];
  }

  if (USE_PLANE) {
    zpw = para_plane_Z[0][0];
    rpw = Quaterniond(para_plane_R[0][3], para_plane_R[0][0], para_plane_R[0][1], para_plane_R[0][2])
              .normalized()
              .toRotationMatrix();
  }

  VectorXd dep = f_manager.getDepthVector();
  VectorXd wei = f_manager.getWeightVector();
  int featureSel = 0;
  int featureDel = 0;
  for (int i = 0; i < f_manager.getFeatureCount(); i++) {
    dep(i) = para_Feature[i][0];
    wei(i) = para_FeatureWeight[i][0];
  }
  f_manager.setWeight(wei);
  f_manager.setDepth(dep);

  if (USE_IMU) td_inertial = para_Td[0][0];
}

bool Estimator::detectFailure() {
  if (f_manager.last_track_num < 40) {
    USE_CAMERA = 0;
    ROS_WARN("Too few features. Turn off visual constraints %d", f_manager.last_track_num);
    return true;
  }
  if (Bas[WINDOW_SIZE].norm() > 2.5) {
    ROS_INFO("Large accelerometer bias estimation %f", Bas[WINDOW_SIZE].norm());
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0) {
    ROS_INFO("Large gyroscope bias estimation %f", Bgs[WINDOW_SIZE].norm());
    return true;
  }
  Vector3d tmp_P = Ps[WINDOW_SIZE];
  if ((tmp_P - last_P).norm() > 5) {
    ROS_INFO("Large translation");
  }
  if (abs(tmp_P.z() - last_P.z()) > 1) {
    ROS_INFO("Large Z-translation");
  }
  Matrix3d tmp_R = Rs[WINDOW_SIZE];
  Matrix3d delta_R = tmp_R.transpose() * last_R;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50) {
    ROS_INFO("Large relative orientation");
  }
  return false;
}

void Estimator::optimizeVIO(bool flagOptimizeWeight) {
  TicToc t_whole, t_prepare;
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(1.0);

  for (int i = 0; i < frame_count + 1; i++) {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    if (USE_IMU) problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);

    if (flagOptimizeWeight) {
      problem.SetParameterBlockConstant(para_Pose[i]);
      if (USE_IMU) problem.SetParameterBlockConstant(para_SpeedBias[i]);
    }
  }

  problem.SetParameterBlockConstant(para_Pose[0]);

  for (int i = 0; i < NUM_OF_CAM; i++) {
    ceres::LocalParameterization *local_parameterization;
    if (ESTIMATE_EXTRINSIC) {
      switch (CAM_EXT_ADJ_TYPE) {
        case CameraExtrinsicAdjustType::ADJUST_CAM_NO_Z:
          local_parameterization = new PoseSubsetParameterization({2, 6});
          break;
        case CameraExtrinsicAdjustType::ADJUST_CAM_ROTATION:
          local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
          break;
        case CameraExtrinsicAdjustType::ADJUST_CAM_TRANSLATION:
          local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
          break;
        case CameraExtrinsicAdjustType::ADJUST_CAM_NO_ROTATION_NO_Z:
          local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
          break;
        default:
          local_parameterization = new PoseSubsetParameterization({});
      }
    } else
      local_parameterization = new PoseLocalParameterization();

    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
    if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation) {
      openExEstimation = 1;
    } else {
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
  }

  if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) {
    ceres::LocalParameterization *local_parameterization;
    if (ESTIMATE_EXTRINSIC_WHEEL) {
      switch (WHEEL_EXT_ADJ_TYPE) {
        case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_Z:
          local_parameterization = new PoseSubsetParameterization({2, 6});
          break;
        case WheelExtrinsicAdjustType::ADJUST_WHEEL_ROTATION:
          local_parameterization = new PoseSubsetParameterization({0, 1, 2, 6});
          break;
        case WheelExtrinsicAdjustType::ADJUST_WHEEL_TRANSLATION:
          local_parameterization = new PoseSubsetParameterization({3, 4, 5, 6});
          break;
        case WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_ROTATION_NO_Z:
          local_parameterization = new PoseSubsetParameterization({2, 3, 4, 5, 6});
          break;
        default:
          local_parameterization = new PoseSubsetParameterization({});
      }
    } else
      local_parameterization = new PoseLocalParameterization();

    problem.AddParameterBlock(para_Ex_Pose_wheel[0], SIZE_POSE, local_parameterization);
    if ((ESTIMATE_EXTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExWheelEstimation) {
      openExWheelEstimation = 1;
    } else {
      problem.SetParameterBlockConstant(para_Ex_Pose_wheel[0]);
    }
    problem.AddParameterBlock(para_Ix_sx_wheel[0], 1);
    problem.AddParameterBlock(para_Ix_sy_wheel[0], 1);
    problem.AddParameterBlock(para_Ix_sw_wheel[0], 1);
    if ((ESTIMATE_INTRINSIC_WHEEL && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openIxEstimation) {
      openIxEstimation = 1;
    } else {
      problem.SetParameterBlockConstant(para_Ix_sx_wheel[0]);
      problem.SetParameterBlockConstant(para_Ix_sy_wheel[0]);
      problem.SetParameterBlockConstant(para_Ix_sw_wheel[0]);
    }
  }

  if (USE_PLANE) {
    ceres::LocalParameterization *local_parameterization = new OrientationSubsetParameterization(std::vector<int>{2});
    problem.AddParameterBlock(para_plane_R[0], SIZE_ROTATION, local_parameterization);
    problem.AddParameterBlock(para_plane_Z[0], 1);
    problem.SetParameterBlockConstant(para_plane_R[0]);
    problem.SetParameterBlockConstant(para_plane_Z[0]);
  }

  problem.AddParameterBlock(para_Td[0], 1);
  if (!ESTIMATE_TD_INERTIAL || Vs[0].norm() < 0.2) problem.SetParameterBlockConstant(para_Td[0]);

  problem.AddParameterBlock(para_Td_wheel[0], 1);
  if (!ESTIMATE_TD_WHEEL || Vs[0].norm() < 0.2) problem.SetParameterBlockConstant(para_Td_wheel[0]);

  if (last_marginalization_info && last_marginalization_info->valid) {
    MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
  }

  if (USE_IMU && !ONLY_INITIAL_WITH_IMU) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      if (pre_integrations[j]->sum_dt > 10.0) continue;
      IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
      problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
  }

  if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      if (pre_integrations_wheel[j]->sum_dt > 10.0) continue;
      WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[j]);
      problem.AddResidualBlock(wheel_factor, NULL, para_Pose[i], para_Pose[j], para_Ex_Pose_wheel[0],
                               para_Ix_sx_wheel[0], para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]);
    }
  }

  if (USE_PLANE) {
    for (int i = 0; i < frame_count; i++) {
      OriginPlaneFactor *origin_plane_factor = new OriginPlaneFactor();
      problem.AddResidualBlock(origin_plane_factor, NULL, para_Pose[i]);
    }
  }

  int f_m_cnt = 0;
  int feature_index = -1;

  if (USE_CAMERA) {
    for (auto &it_per_id : f_manager.feature) {
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (it_per_id.used_num < 4) {
        continue;
      }

      ++feature_index;

      if (USE_GNC) {
        problem.AddParameterBlock(para_FeatureWeight[feature_index], 1);
        if (!flagOptimizeWeight) {
          problem.SetParameterBlockConstant(para_FeatureWeight[feature_index]);
          bool use_regularization = false;
          if (use_regularization) {
            double regularizer = REGULARIZER;
            auto *regularization_factor = new SimpleRegularizationFactor(regularizer);
            problem.AddResidualBlock(regularization_factor, nullptr, para_FeatureWeight[feature_index]);
          }
        } else {
          problem.AddParameterBlock(para_Feature[feature_index], 1);
          problem.SetParameterBlockConstant(para_Feature[feature_index]);
          double regularizer = REGULARIZER;
          double prior_value = para_FeatureWeight[feature_index][0];

          problem.SetParameterUpperBound(para_FeatureWeight[feature_index], 0, 1.0);
          problem.SetParameterLowerBound(para_FeatureWeight[feature_index], 0, 0.0);

          // TODO: change depending on loss
          // double loss_shape = FIXED_SHAPE;
          // auto loss_type = LOSS_TYPE;
          // auto surrogate_loss = createSurrogateLoss(loss_type, loss_shape);
          // double reliability = it_per_id.reliability_model.getReliability();
          // surrogate_loss->updateShape(reliability);
          // double shape = surrogate_loss->getShape();
          // auto *r = new AdaptiveRegularizationFactor(regularizer, shape);
        }
      }

      int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

      Vector3d pts_i = it_per_id.feature_per_frame[0].point;

      for (auto &it_per_frame : it_per_id.feature_per_frame) {
        imu_j++;
        if (imu_i != imu_j) {
          Vector3d pts_j = it_per_frame.point;
          if (USE_GNC) {
            if (USE_SIMPLE_REPROJECTION_FACTORS) {
              AltSimpleReprojectionTwoFrameOneCamFactor *f_td = new AltSimpleReprojectionTwoFrameOneCamFactor(
                  pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
              problem.AddResidualBlock(f_td, NULL, para_Pose[imu_i], para_Pose[imu_j],
                                       para_FeatureWeight[feature_index]);
            } else {
              AltProjectionTwoFrameOneCamFactor *f_td = new AltProjectionTwoFrameOneCamFactor(
                  pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                       para_Feature[feature_index], para_Td[0], para_FeatureWeight[feature_index]);
            }
          } else {
            if (USE_SIMPLE_REPROJECTION_FACTORS) {
              SimpleReprojectionTwoFrameOneCamFactor *f_td = new SimpleReprojectionTwoFrameOneCamFactor(
                  pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
              problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j]);
            } else {
              ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(
                  pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                       para_Feature[feature_index], para_Td[0]);
            }
          }
        }

        if (STEREO && it_per_frame.is_stereo) {
          Vector3d pts_j_right = it_per_frame.pointRight;
          if (imu_i != imu_j) {
            if (USE_GNC) {
              if (USE_SIMPLE_REPROJECTION_FACTORS) {
                AltSimpleReprojectionTwoFrameTwoCamFactor *f = new AltSimpleReprojectionTwoFrameTwoCamFactor(
                    pts_i, pts_j_right, qic[0], tic[0], qic[1], tic[1], para_Feature[feature_index][0]);
                problem.AddResidualBlock(f, NULL, para_Pose[imu_i], para_Pose[imu_j],
                                         para_FeatureWeight[feature_index]);
              } else {
                AltProjectionTwoFrameTwoCamFactor *af = new AltProjectionTwoFrameTwoCamFactor(
                    pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(af, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                         para_Ex_Pose[1], para_Feature[feature_index], para_Td[0],
                                         para_FeatureWeight[feature_index]);
              }
            } else {
              if (USE_SIMPLE_REPROJECTION_FACTORS) {
                SimpleReprojectionTwoFrameTwoCamFactor *f = new SimpleReprojectionTwoFrameTwoCamFactor(
                    pts_i, pts_j_right, qic[0], tic[0], qic[1], tic[1], para_Feature[feature_index][0]);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j]);
              } else {
                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
                    pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                         para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
              }
            }
          } else {
            if (!USE_SIMPLE_REPROJECTION_FACTORS) {
              ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
                  pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index],
                                       para_Td[0]);
            }
          }
        }
        f_m_cnt++;
      }
    }
  }

  ROS_DEBUG("visual measurement count: %d", f_m_cnt);
  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  if (marginalization_flag == KEYFRAME)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
    options.max_solver_time_in_seconds = SOLVER_TIME;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
}

double Estimator::initMu() {
  int feature_index = -1;
  double max_residual = 0.0;
  double loss_shape = FIXED_SHAPE;
  auto loss_type = LOSS_TYPE;

  for (auto &it_per_id : f_manager.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) {
      continue;
    }

    ++feature_index;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        if (USE_SIMPLE_REPROJECTION_FACTORS) {
          AltSimpleReprojectionTwoFrameOneCamFactor *f_td = new AltSimpleReprojectionTwoFrameOneCamFactor(
              pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
          double residual = f_td->ComputeResidual(para_Pose[imu_i], para_Pose[imu_j]);
          if (residual > max_residual) max_residual = residual;
        } else {
          ROS_ERROR("ERROR: use simple factors");
        }
      }
    }
  }

  auto surrogate_loss = createSurrogateLoss(loss_type, loss_shape);

  return surrogate_loss->initializeControlParameter(max_residual);
}

void Estimator::optimization() {
  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(1.0);

  vector2double();

  int outer_iterations = 0;
  if (USE_GNC) {
    double control_parameter = CONVEXITY_INIT;
    bool adapt_control_parameter = static_cast<bool>(ADAPTIVE_GNC);
    double loss_shape = FIXED_SHAPE;
    auto loss_type = LOSS_TYPE;

    auto surrogate_loss = createSurrogateLoss(loss_type, loss_shape);

    if (loss_type == SurrogateLossType::ADAPTIVE && adapt_control_parameter) {
      double adaptive_control_parameter;
      control_parameter = 1.0;
      for (auto &it_per_id : f_manager.feature) {
        double reliability = it_per_id.reliability_model.getReliability();
        surrogate_loss->updateShape(reliability);
        adaptive_control_parameter = surrogate_loss->updateControlParameter();
        if (adaptive_control_parameter > control_parameter) control_parameter = adaptive_control_parameter;
      }
    }

    while (control_parameter >= CONVEXITY_MARGIN) {
      optimizeVIO(false);

      double2vector();

      int feature_index = -1;

      for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4) {
          continue;
        }

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        double reliability = it_per_id.reliability_model.getReliability();
        // if (loss_type == SurrogateLossType::ADAPTIVE) surrogate_loss->updateShape(reliability);

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
          imu_j++;
          if (imu_i != imu_j) {
            Vector3d pts_j = it_per_frame.point;
            if (USE_SIMPLE_REPROJECTION_FACTORS) {
              AltSimpleReprojectionTwoFrameOneCamFactor *f_td = new AltSimpleReprojectionTwoFrameOneCamFactor(
                  pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
              double residual = f_td->ComputeResidual(para_Pose[imu_i], para_Pose[imu_j]);
              if (USE_SEMANTICS && reliability < 0.25)
                para_FeatureWeight[feature_index][0] = reliability;
              else
                surrogate_loss->updateWeight(residual, control_parameter, para_FeatureWeight[feature_index][0]);
            } else {
              AltProjectionTwoFrameOneCamFactor *f_td = new AltProjectionTwoFrameOneCamFactor(
                  pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              double residual = f_td->ComputeResidual(para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                                      para_Feature[feature_index], para_Td[0]);
              if (USE_SEMANTICS && reliability < 0.25)
                para_FeatureWeight[feature_index][0] = reliability;
              else
                surrogate_loss->updateWeight(residual, control_parameter, para_FeatureWeight[feature_index][0]);
            }
          }

          // TODO: add stereo case here (2 cam --> 2 cam)
        }
      }

      control_parameter /= CONVEXITY_UPDATE;
      outer_iterations++;
    }

    for (auto &it_per_id : f_manager.feature) {
      it_per_id.used_num = it_per_id.feature_per_frame.size();
      if (it_per_id.used_num < 4) continue;
      it_per_id.weightNum++;
    }

  } else
    optimizeVIO(false);

  // TODO: update weight here

  double2vector();

  if (frame_count < WINDOW_SIZE) return;

  if (marginalization_flag == KEYFRAME) {
    auto *marginalization_info = new MarginalizationInfo();
    vector2double();

    if (last_marginalization_info && last_marginalization_info->valid) {
      vector<int> drop_set;
      for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }
      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
      ResidualBlockInfo *residual_block_info =
          new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    if (USE_IMU && !ONLY_INITIAL_WITH_IMU) {
      if (pre_integrations[1]->sum_dt < 10.0) {
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);

        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL, vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
            vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }
    //  TODO: add weight to residual here
    if (USE_WHEEL && !ONLY_INITIAL_WITH_WHEEL) {
      if (pre_integrations_wheel[1]->sum_dt < 10.0) {
        WheelFactor *wheel_factor = new WheelFactor(pre_integrations_wheel[1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            wheel_factor, NULL,
            vector<double *>{para_Pose[0], para_Pose[1], para_Ex_Pose_wheel[0], para_Ix_sx_wheel[0],
                             para_Ix_sy_wheel[0], para_Ix_sw_wheel[0], para_Td_wheel[0]},
            vector<int>{0});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    if (USE_PLANE) {
      OriginPlaneFactor *origin_plane_factor = new OriginPlaneFactor();
      ResidualBlockInfo *residual_block_info =
          new ResidualBlockInfo(origin_plane_factor, NULL, vector<double *>{para_Pose[0]}, vector<int>{0});
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    if (USE_CAMERA) {
      {
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature) {
          it_per_id.used_num = it_per_id.feature_per_frame.size();
          if (it_per_id.used_num < 4) continue;

          ++feature_index;

          if (USE_GNC && it_per_id.weight < FEATURE_WEIGHT_THRESHOLD) continue;
          int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
          if (imu_i != 0) continue;

          Vector3d pts_i = it_per_id.feature_per_frame[0].point;

          for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i != imu_j) {
              Vector3d pts_j = it_per_frame.point;
              if (USE_GNC) {
                if (USE_SIMPLE_REPROJECTION_FACTORS) {
                  AltSimpleReprojectionTwoFrameOneCamFactor *f_td = new AltSimpleReprojectionTwoFrameOneCamFactor(
                      pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
                  ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                      f_td, NULL,
                      vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_FeatureWeight[feature_index]},
                      vector<int>{0, 2});
                  marginalization_info->addResidualBlockInfo(residual_block_info);
                } else {
                  AltProjectionTwoFrameOneCamFactor *f_td = new AltProjectionTwoFrameOneCamFactor(
                      pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                      it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                  ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                      f_td, loss_function,
                      vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index],
                                       para_Td[0], para_FeatureWeight[feature_index]},
                      vector<int>{0, 3, 5});
                  marginalization_info->addResidualBlockInfo(residual_block_info);
                }
              } else {
                if (USE_SIMPLE_REPROJECTION_FACTORS) {
                  SimpleReprojectionTwoFrameOneCamFactor *f_td = new SimpleReprojectionTwoFrameOneCamFactor(
                      pts_i, pts_j, qic[0], tic[0], para_Feature[feature_index][0]);
                  ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                      f_td, loss_function, vector<double *>{para_Pose[imu_i], para_Pose[imu_j]}, vector<int>{0});
                  marginalization_info->addResidualBlockInfo(residual_block_info);
                } else {
                  ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(
                      pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                      it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                  ResidualBlockInfo *residual_block_info =
                      new ResidualBlockInfo(f_td, loss_function,
                                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0],
                                                             para_Feature[feature_index], para_Td[0]},
                                            vector<int>{0, 3});
                  marginalization_info->addResidualBlockInfo(residual_block_info);
                }
              }
            }
            if (STEREO && it_per_frame.is_stereo) {
              Vector3d pts_j_right = it_per_frame.pointRight;
              if (imu_i != imu_j) {
                if (USE_GNC) {
                  if (USE_SIMPLE_REPROJECTION_FACTORS) {
                    AltSimpleReprojectionTwoFrameTwoCamFactor *f = new AltSimpleReprojectionTwoFrameTwoCamFactor(
                        pts_i, pts_j_right, qic[0], tic[0], qic[1], tic[1], para_Feature[feature_index][0]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                        f, NULL,
                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_FeatureWeight[feature_index]},
                        vector<int>{0, 2});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                  } else {
                    ProjectionTwoFrameTwoCamAltFactor *af = new ProjectionTwoFrameTwoCamAltFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                        af, loss_function,
                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                                         para_Feature[feature_index], para_Td[0], para_FeatureWeight[feature_index]},
                        vector<int>{0, 4, 6});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                  }
                } else {
                  if (USE_SIMPLE_REPROJECTION_FACTORS) {
                    SimpleReprojectionTwoFrameTwoCamFactor *f = new SimpleReprojectionTwoFrameTwoCamFactor(
                        pts_i, pts_j_right, qic[0], tic[0], qic[1], tic[1], para_Feature[feature_index][0]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                        f, loss_function, vector<double *>{para_Pose[imu_i], para_Pose[imu_j]}, vector<int>{0});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                  } else {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(
                        pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                        f, loss_function,
                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                                         para_Feature[feature_index], para_Td[0]},
                        vector<int>{0, 4});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                  }
                }
              } else {
                if (!USE_SIMPLE_REPROJECTION_FACTORS) {
                  ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(
                      pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                      it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                  ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                      f, loss_function,
                      vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                      vector<int>{2});
                  marginalization_info->addResidualBlockInfo(residual_block_info);
                }
              }
            }
          }
        }
      }
    }

    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

    TicToc t_margin;
    marginalization_info->marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.toc());
    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++) {
      addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      if (USE_IMU) addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
    }
    for (int i = 0; i < NUM_OF_CAM; i++) addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
    addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
    addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
    addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

    addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
    addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];

    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

    addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];

    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info) delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;
  } else {
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1])) {
      MarginalizationInfo *marginalization_info = new MarginalizationInfo();
      vector2double();
      if (last_marginalization_info && last_marginalization_info->valid) {
        vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
          ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
          if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1]) drop_set.push_back(i);
        }
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo *residual_block_info =
            new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      TicToc t_pre_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->preMarginalize();
      ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

      TicToc t_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->marginalize();
      ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= WINDOW_SIZE; i++) {
        if (i == WINDOW_SIZE - 1)
          continue;
        else if (i == WINDOW_SIZE) {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
          if (USE_IMU) addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        } else {
          addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
          if (USE_IMU) addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++) addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

      addr_shift[reinterpret_cast<long>(para_Ex_Pose_wheel[0])] = para_Ex_Pose_wheel[0];
      addr_shift[reinterpret_cast<long>(para_Ix_sx_wheel[0])] = para_Ix_sx_wheel[0];
      addr_shift[reinterpret_cast<long>(para_Ix_sy_wheel[0])] = para_Ix_sy_wheel[0];
      addr_shift[reinterpret_cast<long>(para_Ix_sw_wheel[0])] = para_Ix_sw_wheel[0];

      addr_shift[reinterpret_cast<long>(para_plane_R[0])] = para_plane_R[0];
      addr_shift[reinterpret_cast<long>(para_plane_Z[0])] = para_plane_Z[0];

      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

      addr_shift[reinterpret_cast<long>(para_Td_wheel[0])] = para_Td_wheel[0];

      vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
      if (last_marginalization_info) delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;
    }
  }
}

void Estimator::slideWindow() {
  TicToc t_margin;
  if (marginalization_flag == KEYFRAME) {
    double t_0 = Headers[0];
    back_R0 = Rs[0];
    back_P0 = Ps[0];
    if (frame_count == WINDOW_SIZE) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        Headers[i] = Headers[i + 1];
        Rs[i].swap(Rs[i + 1]);
        Ps[i].swap(Ps[i + 1]);
        if (USE_IMU) {
          std::swap(pre_integrations[i], pre_integrations[i + 1]);

          dt_buffer[i].swap(dt_buffer[i + 1]);
          linear_acceleration_buffer[i].swap(linear_acceleration_buffer[i + 1]);
          angular_velocity_buffer[i].swap(angular_velocity_buffer[i + 1]);

          Vs[i].swap(Vs[i + 1]);
          Bas[i].swap(Bas[i + 1]);
          Bgs[i].swap(Bgs[i + 1]);
        }
        if (USE_WHEEL) {
          std::swap(pre_integrations_wheel[i], pre_integrations_wheel[i + 1]);

          wheel_dt_buffer[i].swap(wheel_dt_buffer[i + 1]);
          wheel_linear_velocity_buffer[i].swap(wheel_linear_velocity_buffer[i + 1]);
          wheel_angular_velocity_buffer[i].swap(wheel_angular_velocity_buffer[i + 1]);
        }
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

      if (USE_IMU) {
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

        delete pre_integrations[WINDOW_SIZE];
        pre_integrations[WINDOW_SIZE] = new IntegrationBase{latest_linear_acceleration, latest_angular_velocity,
                                                            Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

        dt_buffer[WINDOW_SIZE].clear();
        linear_acceleration_buffer[WINDOW_SIZE].clear();
        angular_velocity_buffer[WINDOW_SIZE].clear();
      }

      if (USE_WHEEL) {
        delete pre_integrations_wheel[WINDOW_SIZE];
        pre_integrations_wheel[WINDOW_SIZE] =
            new WheelIntegrationBase{latest_wheel_linear_velocity, latest_wheel_angular_velocity, sx, sy, sw, td_wheel};

        wheel_dt_buffer[WINDOW_SIZE].clear();
        wheel_linear_velocity_buffer[WINDOW_SIZE].clear();
        wheel_angular_velocity_buffer[WINDOW_SIZE].clear();
      }

      if (true || solver_flag == INITIALIZATION) {
        map<double, ImageFrame>::iterator it_0;
        it_0 = image_frame_history.find(t_0);
        for (map<double, ImageFrame>::iterator it = image_frame_history.begin(); it != it_0; ++it) {
          delete it->second.imu_preintegration;
          delete it->second.pre_integration_wheel;
          it->second.imu_preintegration = NULL;
          it->second.pre_integration_wheel = NULL;
        }
        image_frame_history.erase(image_frame_history.begin(), it_0);
      }
      slideWindowOld();
    }
  } else {
    if (frame_count == WINDOW_SIZE) {
      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];

      if (USE_IMU) {
        for (unsigned int i = 0; i < dt_buffer[frame_count].size(); i++) {
          double tmp_dt = dt_buffer[frame_count][i];
          Vector3d tmp_linear_acceleration = linear_acceleration_buffer[frame_count][i];
          Vector3d tmp_angular_velocity = angular_velocity_buffer[frame_count][i];

          pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

          dt_buffer[frame_count - 1].push_back(tmp_dt);
          linear_acceleration_buffer[frame_count - 1].push_back(tmp_linear_acceleration);
          angular_velocity_buffer[frame_count - 1].push_back(tmp_angular_velocity);
        }

        Vs[frame_count - 1] = Vs[frame_count];
        Bas[frame_count - 1] = Bas[frame_count];
        Bgs[frame_count - 1] = Bgs[frame_count];

        delete pre_integrations[WINDOW_SIZE];
        pre_integrations[WINDOW_SIZE] = new IntegrationBase{latest_linear_acceleration, latest_angular_velocity,
                                                            Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

        dt_buffer[WINDOW_SIZE].clear();
        linear_acceleration_buffer[WINDOW_SIZE].clear();
        angular_velocity_buffer[WINDOW_SIZE].clear();
      }
      if (USE_WHEEL) {
        for (unsigned int i = 0; i < wheel_dt_buffer[frame_count].size(); i++) {
          double tmp_dt = wheel_dt_buffer[frame_count][i];
          Vector3d tmp_linear_velocity = wheel_linear_velocity_buffer[frame_count][i];
          Vector3d tmp_angular_velocity = wheel_angular_velocity_buffer[frame_count][i];

          pre_integrations_wheel[frame_count - 1]->push_back(tmp_dt, tmp_linear_velocity, tmp_angular_velocity);

          wheel_dt_buffer[frame_count - 1].push_back(tmp_dt);
          wheel_linear_velocity_buffer[frame_count - 1].push_back(tmp_linear_velocity);
          wheel_angular_velocity_buffer[frame_count - 1].push_back(tmp_angular_velocity);
        }

        delete pre_integrations_wheel[WINDOW_SIZE];
        pre_integrations_wheel[WINDOW_SIZE] =
            new WheelIntegrationBase{latest_wheel_linear_velocity, latest_wheel_angular_velocity, sx, sy, sw, td_wheel};

        wheel_dt_buffer[WINDOW_SIZE].clear();
        wheel_linear_velocity_buffer[WINDOW_SIZE].clear();
        wheel_angular_velocity_buffer[WINDOW_SIZE].clear();
      }
      slideWindowNew();
    }
  }
}

void Estimator::slideWindowNew() {
  sum_of_front++;
  f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth = solver_flag == VIO ? true : false;
  if (shift_depth) {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_R0 * ric[0];
    R1 = Rs[0] * ric[0];
    P0 = back_P0 + back_R0 * tic[0];
    P1 = Ps[0] + Rs[0] * tic[0];
    f_manager.removeBackShiftDepth(R0, P0, R1, P1);
  } else
    f_manager.removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[frame_count];
  T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[index];
  T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame() {
  if (frame_count < 2) return;
  Eigen::Matrix4d curT, prevT, nextT;
  getPoseInWorldFrame(curT);
  getPoseInWorldFrame(frame_count - 1, prevT);
  nextT = curT * (prevT.inverse() * curT);
  map<int, Eigen::Vector3d> predictPts;

  for (auto &it_per_id : f_manager.feature) {
    if (it_per_id.estimated_depth > 0) {
      int firstIndex = it_per_id.start_frame;
      int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
      if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
        double depth = it_per_id.estimated_depth;
        Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
        Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
        Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
        Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
        int ptsIndex = it_per_id.feature_id;
        predictPts[ptsIndex] = pts_cam;
      }
    }
  }
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj,
                                    Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, double depth, Vector3d &uvi,
                                    Vector3d &uvj) {
  Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
  Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &point_ids_to_remove) {
  int feature_index = -1;
  for (auto &it_per_id : f_manager.feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                             depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
      }
      if (STEREO && it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1],
                                               tic[1], depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
        } else {
          double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1],
                                               tic[1], depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
        }
      }
    }
    double ave_err = err / errCnt;
    if (ave_err * FOCAL_LENGTH > 3) point_ids_to_remove.insert(it_per_id.feature_id);
  }
}

void Estimator::fastPredictIMU(double t, const Eigen::Vector3d &linear_acceleration,
                               const Eigen::Vector3d &angular_velocity) {
  double dt = t - fast_inertial_odometry_prediction.latest_time;
  fast_inertial_odometry_prediction.latest_time = t;
  Eigen::Vector3d left_linear_acceleration = fast_inertial_odometry_prediction.latest_orientation *
                                                 (fast_inertial_odometry_prediction.latest_linear_acceleration -
                                                  fast_inertial_odometry_prediction.latest_acc_bias) -
                                             gravity_vector;
  Eigen::Vector3d mid_angular_velocity =
      0.5 * (fast_inertial_odometry_prediction.latest_angular_velocity + angular_velocity) -
      fast_inertial_odometry_prediction.latest_gyro_bias;
  fast_inertial_odometry_prediction.latest_orientation =
      fast_inertial_odometry_prediction.latest_orientation * Utility::deltaQ(mid_angular_velocity * dt);
  Eigen::Vector3d right_linear_acceleration =
      fast_inertial_odometry_prediction.latest_orientation *
          (linear_acceleration - fast_inertial_odometry_prediction.latest_acc_bias) -
      gravity_vector;
  Eigen::Vector3d mid_linear_acceleration = 0.5 * (left_linear_acceleration + right_linear_acceleration);
  fast_inertial_odometry_prediction.latest_position = fast_inertial_odometry_prediction.latest_position +
                                                      dt * fast_inertial_odometry_prediction.latest_velocity +
                                                      0.5 * dt * dt * mid_linear_acceleration;
  fast_inertial_odometry_prediction.latest_velocity =
      fast_inertial_odometry_prediction.latest_velocity + dt * mid_linear_acceleration;
  fast_inertial_odometry_prediction.latest_linear_acceleration = linear_acceleration;
  fast_inertial_odometry_prediction.latest_angular_velocity = angular_velocity;
}

void Estimator::fastPredictWheelOdometry(double t, const Eigen::Vector3d &linear_velocity,
                                         const Eigen::Vector3d &angular_velocity) {
  ROS_DEBUG("Perform fast wheel motion model update");
  double dt = t - fast_wheel_odometry_prediction.latest_time;
  fast_wheel_odometry_prediction.latest_time = t;
  Eigen::Vector3d mid_angular_velocity = 0.5 * fast_wheel_odometry_prediction.latest_sw *
                                         (fast_wheel_odometry_prediction.latest_angular_velocity + angular_velocity);
  Eigen::Vector3d left_linear_velocity =
      fast_wheel_odometry_prediction.latest_orientation * fast_wheel_odometry_prediction.latest_linear_velocity;
  Eigen::Matrix3d latest_sv =
      Eigen::Vector3d(fast_wheel_odometry_prediction.latest_sx, fast_wheel_odometry_prediction.latest_sy, 1)
          .asDiagonal();
  fast_wheel_odometry_prediction.latest_orientation =
      fast_wheel_odometry_prediction.latest_orientation * Utility::deltaQ(mid_angular_velocity * dt);
  fast_wheel_odometry_prediction.latest_velocity =
      0.5 * latest_sv * (fast_wheel_odometry_prediction.latest_orientation * linear_velocity + left_linear_velocity);
  fast_wheel_odometry_prediction.latest_position =
      fast_wheel_odometry_prediction.latest_position + dt * fast_wheel_odometry_prediction.latest_velocity;
  fast_wheel_odometry_prediction.latest_linear_velocity = linear_velocity;
  fast_wheel_odometry_prediction.latest_angular_velocity = angular_velocity;
}

void Estimator::fastPredictPureWheelOdometry(double t, const Eigen::Vector3d &linear_velocity,
                                             const Eigen::Vector3d &angular_velocity, Eigen::Vector3d &position,
                                             Eigen::Quaterniond &orientation, Eigen::Vector3d &velocity) {
  ROS_DEBUG("Perform pure wheel odometry fast prediction");
  static bool first_time = false;
  static Eigen::Quaterniond latest_orientation;
  static Eigen::Vector3d latest_velocity = Eigen::Vector3d::Zero();
  static Eigen::Vector3d latest_position = Eigen::Vector3d::Zero();
  static Eigen::Vector3d latest_linear_velocity = Eigen::Vector3d::Zero();
  static Eigen::Vector3d latest_angular_velocity = Eigen::Vector3d::Zero();
  static double latest_time;
  if (!first_time) {
    first_time = true;
    latest_orientation = fast_wheel_odometry_prediction.latest_orientation;
    latest_velocity = fast_wheel_odometry_prediction.latest_velocity;
    latest_position = fast_wheel_odometry_prediction.latest_position;
    latest_linear_velocity = fast_wheel_odometry_prediction.latest_linear_velocity;
    latest_angular_velocity = fast_wheel_odometry_prediction.latest_angular_velocity;
    latest_time = fast_wheel_odometry_prediction.latest_time;
    std::cout << "Fast wheel odometry predicted initial pose: \n"
              << latest_position.transpose() << std::endl
              << latest_orientation.coeffs().transpose() << std::endl;
  }

  double dt = t - latest_time;
  latest_time = t;
  Eigen::Vector3d mid_angular_velocity =
      0.5 * fast_wheel_odometry_prediction.latest_sw * (latest_angular_velocity + angular_velocity);
  Eigen::Vector3d left_linear_velocity = latest_orientation * latest_linear_velocity;
  Eigen::Matrix3d latest_sv =
      Eigen::Vector3d(fast_wheel_odometry_prediction.latest_sx, fast_wheel_odometry_prediction.latest_sy, 1)
          .asDiagonal();
  latest_orientation = latest_orientation * Utility::deltaQ(mid_angular_velocity * dt);
  latest_velocity = 0.5 * latest_sv * (latest_orientation * linear_velocity + left_linear_velocity);
  latest_position = latest_position + dt * latest_velocity;
  latest_linear_velocity = linear_velocity;
  latest_angular_velocity = angular_velocity;

  position = latest_position;
  orientation = latest_orientation;
  velocity = latest_velocity;
}

void Estimator::updateLatestStates() {
  mInertialPropagate.lock();
  fast_inertial_odometry_prediction.latest_time = Headers[frame_count] + td_inertial;
  fast_inertial_odometry_prediction.latest_position = Ps[frame_count];
  fast_inertial_odometry_prediction.latest_orientation = Rs[frame_count];
  fast_inertial_odometry_prediction.latest_velocity = Vs[frame_count];
  fast_inertial_odometry_prediction.latest_acc_bias = Bas[frame_count];
  fast_inertial_odometry_prediction.latest_gyro_bias = Bgs[frame_count];
  fast_inertial_odometry_prediction.latest_linear_acceleration = latest_linear_acceleration;
  fast_inertial_odometry_prediction.latest_angular_velocity = latest_angular_velocity;
  mBuf.lock();
  queue<pair<double, Eigen::Vector3d>> tmp_accBuf = acc_data_buffer;
  queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyro_data_buffer;
  mBuf.unlock();
  while (!tmp_accBuf.empty()) {
    double t = tmp_accBuf.front().first;
    Eigen::Vector3d linear_acceleration = tmp_accBuf.front().second;
    Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
    fastPredictIMU(t, linear_acceleration, gyr);
    tmp_accBuf.pop();
    tmp_gyrBuf.pop();
  }
  mInertialPropagate.unlock();

  mWheelPropagate.lock();
  fast_wheel_odometry_prediction.latest_time = Headers[frame_count] + td_inertial - td_wheel;
  fast_wheel_odometry_prediction.latest_orientation = Rs[frame_count] * RIO;
  fast_wheel_odometry_prediction.latest_position = Rs[frame_count] * TIO + Ps[frame_count];
  fast_wheel_odometry_prediction.latest_sx = sx;
  fast_wheel_odometry_prediction.latest_sy = sy;
  fast_wheel_odometry_prediction.latest_sw = sw;
  fast_wheel_odometry_prediction.latest_linear_velocity = latest_wheel_linear_velocity;
  fast_wheel_odometry_prediction.latest_angular_velocity = latest_wheel_angular_velocity;
  mWOBuf.lock();
  queue<pair<double, Eigen::Vector3d>> tmp_wheel_velBuf = wheel_odometry_linear_velocity_buffer;
  queue<pair<double, Eigen::Vector3d>> tmp_wheel_gyrBuf = wheel_odometry_angular_velocity_buffer;
  mWOBuf.unlock();
  while (!tmp_wheel_velBuf.empty()) {
    double t = tmp_wheel_velBuf.front().first;
    Eigen::Vector3d vel = tmp_wheel_velBuf.front().second;
    Eigen::Vector3d gyr = tmp_wheel_gyrBuf.front().second;
    fastPredictWheelOdometry(t, vel, gyr);
    tmp_wheel_velBuf.pop();
    tmp_wheel_gyrBuf.pop();
  }
  mWheelPropagate.unlock();
}

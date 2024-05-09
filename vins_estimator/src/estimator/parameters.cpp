/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

double WO_LIN_VEL_XY_N;
double WO_LIN_VEL_Z_N;
double WO_ANG_VEL_XY_N;
double WO_ANG_VEL_Z_N;
double SX;
double SY;
double SW;

double ROLL_N, PITCH_N, ZPW_N;
double ROLL_N_INV, PITCH_N_INV, ZPW_N_INV;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Matrix3d RIO;
Eigen::Vector3d TIO;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_EXTRINSIC_WHEEL;
int ESTIMATE_INTRINSIC_WHEEL;
int ESTIMATE_TD_INERTIAL;
int ESTIMATE_TD_WHEEL;
double TRANSLATION_FAILURE_THRESHOLD;
std::string EX_CALIB_RESULT_PATH;
std::string IN_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string INTRINSIC_ITERATE_PATH;
std::string EXTRINSIC_WHEEL_ITERATE_PATH;
std::string EXTRINSIC_CAM_ITERATE_PATH;
std::string PROCESS_TIME_PATH;
std::string TD_WHEEL_PATH;
std::string TD_PATH;
std::string GROUNDTRUTH_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
std::string WHEEL_TOPIC;
int ROW, COL;
double TIMESHIFT_INERTIAL;
double OFFSET_SIM = 0.0;
double TIMESHIFT_WHEEL;
int NUM_OF_CAM;
int RESIZE;
int DILATING_KERNEL_SIZE = 21;
int STEREO;
int USE_IMU;
int USE_CAMERA;
int USE_SEMANTICS;
int USE_WHEEL;
int USE_PLANE;
int ONLY_INITIAL_WITH_IMU;
int ONLY_INITIAL_WITH_WHEEL;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE_LEFT_RAW_TOPIC, IMAGE_RIGHT_RAW_TOPIC;
std::string IMAGE_LEFT_SEMANTIC_TOPIC, IMAGE_RIGHT_SEMANTIC_TOPIC;
std::string FEATURE0_TOPIC, FEATURE1_TOPIC;
std::string GROUNDTRUTH_TOPIC;
std::string SEMANTIC_PALETTE;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_RAW_TRACK;
int SHOW_SEMANTIC_TRACK;
int FLOW_BACK;

int PREDICT_BY_WHEELS;
int USE_SIMPLE_REPROJECTION_FACTORS = 0;
double VO_FAILURE_WEIGHT = 1.0;

int USE_GNC = 1;
int ADAPTIVE_GNC = 0;
double FEATURE_WEIGHT_THRESHOLD = 0.1;

double REGULARIZER;
int SHOW_IMAGE_WEIGHT;
double MAX_DEPTH = 1000.0;

SurrogateLossType LOSS_TYPE = SurrogateLossType::LECLERC;

double FIXED_SHAPE;
double CONVEXITY_INIT;
double CONVEXITY_MARGIN;
double CONVEXITY_UPDATE;

CameraExtrinsicAdjustType CAM_EXT_ADJ_TYPE;
WheelExtrinsicAdjustType WHEEL_EXT_ADJ_TYPE;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name) {
  T ans;
  if (n.getParam(name, ans)) {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(const std::string &config_file) {
  FILE *fh = fopen(config_file.c_str(), "r");
  if (fh == NULL) {
    ROS_WARN("config_file dosen't exist; wrong config_file path");
    ROS_BREAK();
    return;
  }
  fclose(fh);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  fsSettings["image_left_raw_topic"] >> IMAGE_LEFT_RAW_TOPIC;
  fsSettings["image_right_raw_topic"] >> IMAGE_RIGHT_RAW_TOPIC;

  fsSettings["image_left_semantic_topic"] >> IMAGE_LEFT_SEMANTIC_TOPIC;
  fsSettings["image_right_semantic_topic"] >> IMAGE_RIGHT_SEMANTIC_TOPIC;

  fsSettings["feature0_topic"] >> FEATURE0_TOPIC;
  fsSettings["feature1_topic"] >> FEATURE1_TOPIC;
  fsSettings["groundtruth_topic"] >> GROUNDTRUTH_TOPIC;

  MAX_CNT = fsSettings["max_cnt"];
  MIN_DIST = fsSettings["min_dist"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_RAW_TRACK = fsSettings["show_raw_track"];
  SHOW_SEMANTIC_TRACK = fsSettings["show_semantic_track"];
  FLOW_BACK = fsSettings["flow_back"];

  MULTIPLE_THREAD = fsSettings["multiple_thread"];

  MAX_DEPTH = fsSettings["max_depth"];

  USE_CAMERA = fsSettings["camera"];
  printf("USE_CAMERA: %d\n", USE_CAMERA);

  USE_SEMANTICS = fsSettings["semantics"];
  printf("USE_SEMANTICS: %d\n", USE_SEMANTICS);

  USE_IMU = fsSettings["imu"];
  printf("USE_IMU: %d\n", USE_IMU);

  USE_WHEEL = fsSettings["wheel"];
  printf("USE_WHEEL: %d\n", USE_WHEEL);

  ONLY_INITIAL_WITH_IMU = fsSettings["no_inertial_constraints"];
  printf("ONLY_INITIAL_WITH_IMU: %d\n", ONLY_INITIAL_WITH_IMU);

  ONLY_INITIAL_WITH_WHEEL = fsSettings["no_wheel_constraints"];
  printf("ONLY_INITIAL_WITH_WHEEL: %d\n", ONLY_INITIAL_WITH_WHEEL);

  USE_PLANE = fsSettings["plane"];
  printf("USE_PLANE: %d\n", USE_PLANE);

  PREDICT_BY_WHEELS = fsSettings["predict_by_wheels"];
  printf("PREDICT_BY_WHEELS: %d\n", PREDICT_BY_WHEELS);

  RESIZE = fsSettings["resize"];
  printf("RESIZE: %d\n", RESIZE);

  DILATING_KERNEL_SIZE = fsSettings["dilating_kernel_size"];
  printf("DILATING_KERNEL_SIZE: %d\n", DILATING_KERNEL_SIZE);

  fsSettings["semantic_palette"] >> SEMANTIC_PALETTE;
  printf("SEMANTIC_PALETTE: %s\n", SEMANTIC_PALETTE.c_str());

  if (USE_IMU) {
    fsSettings["imu_topic"] >> IMU_TOPIC;
    printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
  }

  fsSettings["output_path"] >> OUTPUT_FOLDER;

  {
    PROCESS_TIME_PATH = OUTPUT_FOLDER + "/process_time.csv";
    std::ofstream fout(PROCESS_TIME_PATH, std::ios::out);
    fout.close();
  }

  if (USE_WHEEL) {
    fsSettings["wheel_topic"] >> WHEEL_TOPIC;
    printf("WHEEL_TOPIC: %s\n", WHEEL_TOPIC.c_str());

    WO_LIN_VEL_XY_N = fsSettings["wheel_linear_velocity_xy_noise_sigma"];
    WO_LIN_VEL_Z_N = fsSettings["wheel_linear_velocity_z_noise_sigma"];
    WO_ANG_VEL_XY_N = fsSettings["wheel_angular_velocity_xy_noise_sigma"];
    WO_ANG_VEL_Z_N = fsSettings["wheel_angular_velocity_z_noise_sigma"];
    std::cout << "WO_LIN_VEL_XY_N " << WO_LIN_VEL_XY_N << std::endl;
    std::cout << "WO_LIN_VEL_Z_N " << WO_LIN_VEL_Z_N << std::endl;
    std::cout << "WO_ANG_VEL_XY_N " << WO_ANG_VEL_XY_N << std::endl;
    std::cout << "WO_ANG_VEL_Z_N " << WO_ANG_VEL_Z_N << std::endl;

    SX = static_cast<double>(fsSettings["sx"]);
    SY = static_cast<double>(fsSettings["sy"]);
    SW = static_cast<double>(fsSettings["sw"]);
    ESTIMATE_EXTRINSIC_WHEEL = fsSettings["estimate_wheel_extrinsic"];
    if (ESTIMATE_EXTRINSIC_WHEEL == 2) {
      ROS_WARN("have no prior about wheel extrinsic param, calibrate extrinsic param");
      RIO = Eigen::Matrix3d::Identity();
      TIO = Eigen::Vector3d::Zero();
      EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    } else {
      if (ESTIMATE_EXTRINSIC_WHEEL == 1) {
        ROS_WARN(" Optimize wheel extrinsic param around initial guess!");
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
      }
      if (ESTIMATE_EXTRINSIC_WHEEL == 0) ROS_WARN(" fix extrinsic param ");

      cv::Mat cv_T;
      fsSettings["body_T_wheel"] >> cv_T;
      Eigen::Matrix4d T;
      cv::cv2eigen(cv_T, T);
      RIO = T.block<3, 3>(0, 0);
      TIO = T.block<3, 1>(0, 3);
      Eigen::Quaterniond QIO(RIO);
      QIO.normalize();
      RIO = QIO.toRotationMatrix();
    }
    if (ESTIMATE_EXTRINSIC_WHEEL) {
      EXTRINSIC_WHEEL_ITERATE_PATH = OUTPUT_FOLDER + "/extrinsic_iterate_wheel.csv";
      std::ofstream fout(EXTRINSIC_WHEEL_ITERATE_PATH, std::ios::out);
      fout.close();
    }
    if (ESTIMATE_EXTRINSIC_WHEEL) {
      int extrinsic_type = static_cast<int>(fsSettings["extrinsic_type_wheel"]);
      switch (extrinsic_type) {
        case 0:
          WHEEL_EXT_ADJ_TYPE = WheelExtrinsicAdjustType::ADJUST_WHEEL_ALL;
          ROS_INFO("adjust translation and rotation of cam extrinsic");
          break;
        case 1:
          WHEEL_EXT_ADJ_TYPE = WheelExtrinsicAdjustType::ADJUST_WHEEL_TRANSLATION;
          ROS_INFO("adjust only translation of cam extrinsic");
          break;
        case 2:
          WHEEL_EXT_ADJ_TYPE = WheelExtrinsicAdjustType::ADJUST_WHEEL_ROTATION;
          ROS_INFO("adjust only rotation of cam extrinsic");
          break;
        case 3:
          WHEEL_EXT_ADJ_TYPE = WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_Z;
          ROS_INFO("adjust without Z of translation of wheel extrinsic");
          break;
        case 4:
          WHEEL_EXT_ADJ_TYPE = WheelExtrinsicAdjustType::ADJUST_WHEEL_NO_ROTATION_NO_Z;
          ROS_INFO("adjust without rotation and Z of translation of wheel extrinsic");
          break;
        default:
          ROS_WARN("the extrinsic type range from 0 to 4");
      }
    }

    ESTIMATE_INTRINSIC_WHEEL = static_cast<int>(fsSettings["estimate_wheel_intrinsic"]);
    if (ESTIMATE_INTRINSIC_WHEEL == 2) {
      ROS_WARN("have no prior about wheel intrinsic param, calibrate intrinsic param");
      IN_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/intrinsic_parameter.csv";
      INTRINSIC_ITERATE_PATH = OUTPUT_FOLDER + "/intrinsic_iterate.csv";
      std::ofstream fout(INTRINSIC_ITERATE_PATH, std::ios::out);
      fout.close();
    } else {
      if (ESTIMATE_INTRINSIC_WHEEL == 1) {
        ROS_WARN(" Optimize wheel intrinsic param around initial guess!");
        IN_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/intrinsic_parameter.csv";
        INTRINSIC_ITERATE_PATH = OUTPUT_FOLDER + "/intrinsic_iterate.csv";
        std::ofstream fout(INTRINSIC_ITERATE_PATH, std::ios::out);
        fout.close();
      }
      if (ESTIMATE_INTRINSIC_WHEEL == 0) ROS_WARN(" fix intrinsic param ");
    }
  }

  if (USE_PLANE) {
    ROLL_N = static_cast<double>(fsSettings["roll_n"]);
    PITCH_N = static_cast<double>(fsSettings["pitch_n"]);
    ZPW_N = static_cast<double>(fsSettings["zpw_n"]);
    ROLL_N_INV = 1.0 / ROLL_N;
    PITCH_N_INV = 1.0 / PITCH_N;
    ZPW_N_INV = 1.0 / ZPW_N;
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
  }

  USE_GNC = fsSettings["gnc"];
  printf("USE_GNC: %d\n", USE_GNC);

  int loss_type_parameter = fsSettings["loss"];
  switch (loss_type_parameter) {
    case 0:
      LOSS_TYPE = SurrogateLossType::L1;
      break;
    case 1:
      LOSS_TYPE = SurrogateLossType::GM;
      break;
    case 2:
      LOSS_TYPE = SurrogateLossType::LECLERC;
      break;
    case 3:
      LOSS_TYPE = SurrogateLossType::TLS;
      break;
    case 4:
      LOSS_TYPE = SurrogateLossType::ADAPTIVE;
      break;
    default:
      std::cerr << "Wrong loss type\n";
  }
  printf("LOSS_TYPE: %d\n", loss_type_parameter);

  ADAPTIVE_GNC = fsSettings["adaptive_gnc"];
  if (loss_type_parameter != 4) ADAPTIVE_GNC = 0;
  printf("ADAPTIVE_GNC: %d\n", ADAPTIVE_GNC);

  SHOW_IMAGE_WEIGHT = fsSettings["show_image_feat_weight"];
  printf("SHOW_IMAGE_WEIGHT: %d\n", SHOW_IMAGE_WEIGHT);

  if (USE_GNC) {
    FIXED_SHAPE = fsSettings["fixed_shape"];
    printf("FIXED_SHAPE: %f\n", FIXED_SHAPE);

    CONVEXITY_INIT = fsSettings["convexity_init"];
    printf("CONVEXITY_INIT: %f\n", CONVEXITY_INIT);

    CONVEXITY_MARGIN = fsSettings["convexity_margin"];
    printf("CONVEXITY_MARGIN: %f\n", CONVEXITY_MARGIN);

    CONVEXITY_UPDATE = fsSettings["convexity_update"];
    printf("CONVEXITY_UPDATE: %f\n", CONVEXITY_UPDATE);

    FEATURE_WEIGHT_THRESHOLD = fsSettings["feature_weight_threshold"];

    REGULARIZER = fsSettings["regularizer"];
  }

  SOLVER_TIME = fsSettings["max_solver_time"];
  NUM_ITERATIONS = fsSettings["max_num_iterations"];
  MIN_PARALLAX = fsSettings["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

  VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
  GROUNDTRUTH_PATH = OUTPUT_FOLDER + "/groundtruth.csv";
  std::cout << "result path " << VINS_RESULT_PATH << std::endl;
  std::cout << "groundtruth path " << GROUNDTRUTH_PATH << std::endl;
  std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
  fout.close();
  std::ofstream fout2(GROUNDTRUTH_PATH, std::ios::out);
  fout2.close();

  ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
  if (ESTIMATE_EXTRINSIC == 2) {
    ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
    RIC.push_back(Eigen::Matrix3d::Identity());
    TIC.push_back(Eigen::Vector3d::Zero());
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
  } else {
    if (ESTIMATE_EXTRINSIC == 1) {
      ROS_WARN(" Optimize extrinsic param around initial guess!");
      EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    if (ESTIMATE_EXTRINSIC == 0) ROS_WARN(" fix extrinsic param ");

    cv::Mat cv_T;
    fsSettings["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
    Eigen::Quaterniond QIC(RIC[0]);
    QIC.normalize();
    RIC[0] = QIC.toRotationMatrix();
  }
  if (ESTIMATE_EXTRINSIC) {
    int extrinsic_type = static_cast<int>(fsSettings["extrinsic_type"]);
    switch (extrinsic_type) {
      case 0:
        CAM_EXT_ADJ_TYPE = CameraExtrinsicAdjustType ::ADJUST_CAM_ALL;
        ROS_INFO("adjust rotation and translation of cam extrinsic");
        break;
      case 1:
        CAM_EXT_ADJ_TYPE = CameraExtrinsicAdjustType ::ADJUST_CAM_TRANSLATION;
        ROS_INFO("adjust only translation of cam extrinsic");
        break;
      case 2:
        CAM_EXT_ADJ_TYPE = CameraExtrinsicAdjustType ::ADJUST_CAM_ROTATION;
        ROS_INFO("adjust only rotation of cam extrinsic");
        break;
      case 3:
        CAM_EXT_ADJ_TYPE = CameraExtrinsicAdjustType ::ADJUST_CAM_NO_Z;
        ROS_INFO("adjust without Z of translation of cam extrinsic");
        break;
      case 4:
        CAM_EXT_ADJ_TYPE = CameraExtrinsicAdjustType ::ADJUST_CAM_NO_ROTATION_NO_Z;
        ROS_INFO("adjust without rotation and Z of translation of cam extrinsic");
        break;
      default:
        ROS_WARN("the extrinsic type range from 0 to 4");
    }
  }
  if (ESTIMATE_EXTRINSIC) {
    EXTRINSIC_CAM_ITERATE_PATH = OUTPUT_FOLDER + "/extrinsic_iterate_cam.csv";
    std::ofstream foutD(EXTRINSIC_CAM_ITERATE_PATH, std::ios::out);
    foutD.close();
  }

  NUM_OF_CAM = fsSettings["number_of_cameras"];
  printf("camera number %d\n", NUM_OF_CAM);

  if (NUM_OF_CAM != 1 && NUM_OF_CAM != 2) {
    printf("number_of_cameras should be 1 or 2\n");
    assert(0);
  }

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);

  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  std::string cam0Path = configPath + "/" + cam0Calib;
  CAM_NAMES.push_back(cam0Path);

  if (NUM_OF_CAM == 2) {
    STEREO = 1;
    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib;
    CAM_NAMES.push_back(cam1Path);

    cv::Mat cv_T;
    fsSettings["body_T_cam1"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
    Eigen::Quaterniond QIC(RIC[1]);
    QIC.normalize();
    RIC[1] = QIC.toRotationMatrix();
  }
  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  TRANSLATION_FAILURE_THRESHOLD = fsSettings["translation_failure_threshold"];
  std::cout << "TRANSLATION_FAILURE_THRESHOLD: " << TRANSLATION_FAILURE_THRESHOLD << std::endl;

  TIMESHIFT_INERTIAL = fsSettings["td_inertial"];
  OFFSET_SIM = fsSettings["offset"];
  ESTIMATE_TD_INERTIAL = fsSettings["estimate_td_inertial"];
  if (ESTIMATE_TD_INERTIAL) {
    ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TIMESHIFT_INERTIAL);
    TD_PATH = OUTPUT_FOLDER + "/td_inertial.csv";
    std::ofstream fout(PROCESS_TIME_PATH, std::ios::out);
    fout.close();
  } else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TIMESHIFT_INERTIAL);

  TIMESHIFT_WHEEL = fsSettings["td_wheel"];
  ESTIMATE_TD_WHEEL = fsSettings["estimate_td_wheel"];
  if (ESTIMATE_TD_WHEEL) {
    ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TIMESHIFT_WHEEL);
    TD_WHEEL_PATH = OUTPUT_FOLDER + "/td_wheel.csv";
    std::ofstream fout(TD_WHEEL_PATH, std::ios::out);
    fout.close();
  } else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TIMESHIFT_WHEEL);

  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  ROS_INFO("ROW: %d COL: %d ", ROW, COL);

  if (!USE_IMU) {
    ESTIMATE_EXTRINSIC = 0;
    printf("fix extrinsic param; no time offset calibration\n");
  }
  if (!USE_WHEEL) {
    ESTIMATE_EXTRINSIC_WHEEL = 0;
    ESTIMATE_INTRINSIC_WHEEL = 0;
    ESTIMATE_TD_WHEEL = 0;
    printf("no wheel, fix extrinsic and intrinsic param; no time offset calibration\n");
  }

  fsSettings.release();
}

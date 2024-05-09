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

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <stdio.h>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> image_left_raw_buffer;
queue<sensor_msgs::ImageConstPtr> image_right_raw_buffer;
queue<sensor_msgs::ImageConstPtr> image_left_semantic_buffer;
queue<sensor_msgs::ImageConstPtr> image_right_semantic_buffer;
std::mutex m_buf;

void img_left_raw_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  m_buf.lock();
  image_left_raw_buffer.push(img_msg);
  m_buf.unlock();
}

void img_right_raw_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  m_buf.lock();
  image_right_raw_buffer.push(img_msg);
  m_buf.unlock();
}

void img_left_pair_callback(const sensor_msgs::ImageConstPtr &raw_img_msg,
                            const sensor_msgs::ImageConstPtr &semantic_img_msg) {
  m_buf.lock();
  image_left_raw_buffer.push(raw_img_msg);
  m_buf.unlock();

  m_buf.lock();
  image_left_semantic_buffer.push(semantic_img_msg);
  m_buf.unlock();
}

cv::Mat getRawImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "rgb8") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "rgb8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else if (img_msg->encoding == "bgra8") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "bgra8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }

  // TODO: change K as well
  cv::Mat img = ptr->image.clone();
  if (RESIZE) {
    double x_scale = 0.5;
    double y_scale = 0.5;
    cv::resize(img, img, cv::Size(), x_scale, y_scale);
  }
  return img;
}

cv::Mat getSemanticImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  sensor_msgs::Image img;
  img.header = img_msg->header;
  img.height = img_msg->height;
  img.width = img_msg->width;
  img.is_bigendian = img_msg->is_bigendian;
  img.step = img_msg->step;
  img.data = img_msg->data;
  img.encoding = "mono8";
  ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

  // TODO: RENAME
  cv::Mat image = ptr->image.clone();
  // if (RESIZE) {
  //   double x_scale = 0.5;
  //   double y_scale = 0.5;
  //   cv::resize(image, image, cv::Size(), x_scale, y_scale);
  // }
  return image;
}

void sync_process() {
  while (true) {
    while (image_left_raw_buffer.size() > 3) image_left_raw_buffer.pop();
    while (image_right_raw_buffer.size() > 3) image_right_raw_buffer.pop();
    while (image_left_semantic_buffer.size() > 3) image_left_semantic_buffer.pop();
    while (image_right_semantic_buffer.size() > 3) image_right_semantic_buffer.pop();
    if (STEREO) {
      cv::Mat left_image_raw, right_image_raw;
      // TODO: conditional declaration
      cv::Mat left_image_semantic, right_image_semantic;
      auto left_image_pair = std::make_pair(cv::Mat(), cv::Mat());
      auto right_image_pair = std::make_pair(cv::Mat(), cv::Mat());
      std_msgs::Header header;
      double time = 0;
      m_buf.lock();
      if (!image_left_raw_buffer.empty() && !image_right_raw_buffer.empty()) {
        double time0 = image_left_raw_buffer.front()->header.stamp.toSec();
        double time1 = image_right_raw_buffer.front()->header.stamp.toSec();
        if (time0 < time1) {
          image_left_raw_buffer.pop();
          image_left_semantic_buffer.pop();
          printf("throw img0\n");
        } else if (time0 > time1) {
          image_right_raw_buffer.pop();
          image_right_semantic_buffer.pop();
          printf("throw img1\n");
        } else {
          time = image_left_raw_buffer.front()->header.stamp.toSec();
          header = image_left_raw_buffer.front()->header;
          left_image_raw = getRawImageFromMsg(image_left_raw_buffer.front());
          left_image_pair.first = left_image_raw;
          image_left_raw_buffer.pop();
          right_image_raw = getRawImageFromMsg(image_right_raw_buffer.front());
          right_image_pair.first = right_image_raw;
          image_right_raw_buffer.pop();
          if (USE_SEMANTICS) {
            left_image_semantic = getSemanticImageFromMsg(image_left_semantic_buffer.front());
            left_image_pair.second = left_image_semantic;
            image_left_semantic_buffer.pop();
            right_image_semantic = getSemanticImageFromMsg(image_right_semantic_buffer.front());
            right_image_pair.second = right_image_semantic;
            image_right_semantic_buffer.pop();
          }
        }
      }
      m_buf.unlock();
      if (!left_image_raw.empty()) estimator.processInputImages(time, left_image_pair, right_image_pair);
    } else {
      cv::Mat image_raw, image_semantic;
      auto image_pair = std::make_pair(cv::Mat(), cv::Mat());
      double raw_image_timestamp = 0.0;
      m_buf.lock();
      if (!image_left_raw_buffer.empty()) {
        raw_image_timestamp = image_left_raw_buffer.front()->header.stamp.toSec();
        image_raw = getRawImageFromMsg(image_left_raw_buffer.front());
        image_pair.first = image_raw;
        image_left_raw_buffer.pop();
      }
      if (USE_SEMANTICS && !image_left_semantic_buffer.empty()) {
        image_semantic = getSemanticImageFromMsg(image_left_semantic_buffer.front());
        image_pair.second = image_semantic;
        image_left_semantic_buffer.pop();
      }
      m_buf.unlock();
      if (!image_raw.empty()) estimator.processInputImages(raw_image_timestamp, image_pair);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  double t = imu_msg->header.stamp.toSec();
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Vector3d acc(dx, dy, dz);
  Vector3d gyr(rx, ry, rz);
  estimator.inputIMU(t, acc, gyr);
}

void wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
  double t = odom_msg->header.stamp.toSec();
  double vx = odom_msg->twist.twist.linear.x;
  double vy = odom_msg->twist.twist.linear.y;
  double vz = odom_msg->twist.twist.linear.z;
  double rx = odom_msg->twist.twist.angular.x;
  double ry = odom_msg->twist.twist.angular.y;
  double rz = odom_msg->twist.twist.angular.z;
  Vector3d linear_velocity(vx, vy, vz);
  Vector3d angular_velocity(rx, ry, rz);
  estimator.inputWheel(t, linear_velocity, angular_velocity);
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg) {
  FeatureFrame featureFrame;
  for (unsigned int i = 0; i < feature_msg->points.size(); i++) {
    int feature_id = feature_msg->channels[0].values[i];
    int camera_id = feature_msg->channels[1].values[i];
    double x = feature_msg->points[i].x;
    double y = feature_msg->points[i].y;
    double z = feature_msg->points[i].z;
    double p_u = feature_msg->channels[2].values[i];
    double p_v = feature_msg->channels[3].values[i];
    double velocity_x = feature_msg->channels[4].values[i];
    double velocity_y = feature_msg->channels[5].values[i];
    if (feature_msg->channels.size() > 5) {
      double gx = feature_msg->channels[6].values[i];
      double gy = feature_msg->channels[7].values[i];
      double gz = feature_msg->channels[8].values[i];
      pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
    }
    ROS_ASSERT(z == 1);
    Feature xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }
  double t = feature_msg->header.stamp.toSec();
  estimator.inputFeature(t, featureFrame);
  return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg) {
  if (restart_msg->data == true) {
    ROS_WARN("restart the estimator!");
    m_buf.lock();
    while (!feature_buf.empty()) feature_buf.pop();
    while (!imu_buf.empty()) imu_buf.pop();
    m_buf.unlock();
    estimator.clearState();
    estimator.setParameter();
  }
  return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg) {
  if (switch_msg->data == true) {
    estimator.changeSensorType(1, STEREO);
  } else {
    estimator.changeSensorType(0, STEREO);
  }
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg) {
  if (switch_msg->data == true) {
    estimator.changeSensorType(USE_IMU, 1);
  } else {
    estimator.changeSensorType(USE_IMU, 0);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  if (argc != 2) {
    printf(
        "please intput: rosrun vins vins_node [config file] \n"
        "for example: rosrun vins vins_node "
        "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
    return 1;
  }

  string config_file = argv[1];
  printf("config_file: %s\n", argv[1]);

  readParameters(config_file);
  estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
  ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

  ROS_WARN("waiting for sensor data...");

  registerPub(n);

  message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(n, IMAGE_LEFT_RAW_TOPIC, 1);
      message_filters::Subscriber<sensor_msgs::Image> semantic_image_subscriber(n, IMAGE_LEFT_SEMANTIC_TOPIC, 1);
      using synchronization_policy =
          message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
      message_filters::Synchronizer<synchronization_policy> sync(synchronization_policy(10), raw_image_subscriber,
                                                                 semantic_image_subscriber);

  ros::Subscriber raw_left_image_subscriber, raw_right_image_subscriber, imu_subscriber, wheel_odometry_subscriber;

  if (STEREO) {
    if (USE_SEMANTICS) {
      throw std::runtime_error("Semantic mode for stereo VIO is not implemented yet! Bye!");
    } else {
      raw_left_image_subscriber = n.subscribe(IMAGE_LEFT_RAW_TOPIC, 3, img_left_raw_callback);
      raw_right_image_subscriber = n.subscribe(IMAGE_RIGHT_RAW_TOPIC, 3, img_right_raw_callback);
    }
  } else {
    if (USE_SEMANTICS) {
      sync.registerCallback(boost::bind(&img_left_pair_callback, _1, _2));
    } else
      raw_left_image_subscriber = n.subscribe(IMAGE_LEFT_RAW_TOPIC, 3, img_left_raw_callback);
  }

  if (USE_IMU) {
    imu_subscriber = n.subscribe(IMU_TOPIC, 300, imu_callback, ros::TransportHints().tcpNoDelay());
  }
  if (USE_WHEEL)
    wheel_odometry_subscriber = n.subscribe(WHEEL_TOPIC, 2000, wheel_callback, ros::TransportHints().tcpNoDelay());

  ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 3, feature_callback);
  ros::Subscriber sub_restart = n.subscribe("/vins_restart", 1, restart_callback);
  ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 1, imu_switch_callback);
  ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 1, cam_switch_callback);

  std::thread sync_thread{sync_process};
  ros::spin();
  return 0;
}

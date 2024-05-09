cmake_minimum_required(VERSION 2.8.3)
project(robust_vins)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g -march=native")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -Wall -g -march=native")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        camera_models
        vins
        image_transport
        pcl_conversions
        pcl_ros
        rosbag)
#set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
#template sophus
#set(Sophus_DIR "/home/td/slam/install/sophus/share/sophus/cmake/")
find_package(Sophus REQUIRED)
# include_directories(/home/td/slam/install/sophus/include)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)


catkin_package(
        CATKIN_DEPENDS roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        camera_models
        vins
        pcl_conversions
        pcl_ros
        image_transport
        rosbag
)

cmake_minimum_required(VERSION 2.8.3)
project(vins)
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
        INCLUDE_DIRS src
        LIBRARIES vins_lib
        CATKIN_DEPENDS roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        camera_models
        pcl_conversions
        pcl_ros
        image_transport
        rosbag
)

catkin_install_python(PROGRAMS ../scripts/semantic_segmentation_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

set(FMT_LIBS fmt)

add_library(vins_lib
        src/estimator/parameters.cpp
        src/estimator/estimator.cpp
        src/estimator/feature_manager.cpp
        src/estimator/surrogate_loss.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/pose_subset_parameterization.cpp
        src/factor/orientation_local_parameterization.cpp
        src/factor/orientation_subset_parameterization.cpp
        src/factor/projectionTwoFrameOneCamFactor.cpp
        src/factor/projectionTwoFrameTwoCamFactor.cpp
        src/factor/projectionOneFrameTwoCamFactor.cpp
        src/factor/simpleReprojectionTwoFrameOneCamFactor.cpp
        src/factor/simpleReprojectionTwoFrameTwoCamFactor.cpp
        src/factor/altProjectionTwoFrameOneCamFactor.cpp
        src/factor/altProjectionTwoFrameTwoCamFactor.cpp
        src/factor/altSimpleReprojectionTwoFrameOneCamFactor.cpp
        src/factor/altSimpleReprojectionTwoFrameTwoCamFactor.cpp
        src/factor/marginalization_factor.cpp
        src/reliability_model/reliability_model.cpp
        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp
        src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${FMT_LIBS})

add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib)

add_executable(bag_writer src/tool/bagwriter.cpp)
target_link_libraries(bag_writer ${catkin_LIBRARIES})


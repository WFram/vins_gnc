# VINS-GNC

This package implements Visual-Inertial Navigation System (VINS) applied to mobile robots working in dynamic
environment. The implementation is based on [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). Also, it's enhanced by wheel odometry based on [VIW-Fusion](https://github.com/TouchDeeper/VIW-Fusion).

Original paper: [Robust Visual-Inertial Odometry for Ground Robots in Dynamic Environments](https://ieeexplore.ieee.org/document/10168408).

## 1. Installation

Create a catkin workspace where the package will be built.
In order to simplify deployment process, Dockerfile is provided. Build an image and run the container:

```
cd docker/ubuntu
sudo usermod -aG docker $USER
bash docker_build.sh
xhost +local:docker
bash docker_run.sh /path/to/workspace
```

Inside a docker container, run the following commands to build catkin workspace:

```
conda init
source .bashrc
conda activate vins_env

source /opt/ros/noetic/setup.bash

catkin init
catkin config --extend $(rospack find roscpp | sed 's|/share/roscpp$||')
catkin config --merge-devel
catkin config -DCMAKE_BUILD_TYPE=Release -DCATKIN_PYTHON_EXECUTABLE=$(which python)

cd src
git clone https://github.com/WFram/Robust-VINS.git
cd $HOME

catkin build vins
source devel/setup.bash
```

## 2. Run

## 2.1. [VIODE](https://zenodo.org/records/4568610) dataset

Run the following commands:

```
# 1st shell (visualizer)

roslaunch vins vins_rviz.launch

# 2st shell (node)
roslaunch vins viode_vio.launch

# 3rd shell (bag)
rosbag play /path/to/bag
```

## 2.2. Run with semantic segmentation

First, NN model needs to be provided. Can be downloaded from [mmsegmentation](https://github.com/open-mmlab/mmsegmentation) storage.

```
wget -O /path/to/config/nn_model.pth https://download.openmmlab.com/mmsegmentation/v0.5/segformer/segformer_mit-b0_512x512_160k_ade20k/segformer_mit-b0_512x512_160k_ade20k_20210726_101530-8ffa8fda.pth
```

Second, parameter `semantics` in YAML config file has to be set. Finally, VINS node must be run with the additional argument:

```
roslaunch vins <LAUNCH_FILE> semantics:=true
```


## 3. Parameter description

+ `camera`

  Use visual constraints in optimization (turn off only for inertial integration debug)

+ `semantics`

  Use semantic information provided

+ `imu`

  Use inertial integration and constraints in optimization

+ `no_inertial_constraints`

  Use IMU data only for pose prediction. No inertial constraints added into optimization. Not recommended to set

+ `wheel`

  Use wheel odometry readings provided (for wheeled mobile robots)

+ `no_wheel_constraints`

  Use wheel odometry data only for pose prediction. No wheel constraints added into optimization. Not recommended to set

+ `plane`

  Use plane constraint based on planar motion. Reduced accumulated drift along vertical axis

+ `number_of_cameras`

  1: mono setup
  2: stereo setup

+ `semantic_palette`

  Dataset which was used to train a network, whose checkpoints are provided. The only possible options are: ade20k, cityscapes, pascal_voc

+ `predict_by_wheels`

  If true, then instead of inertial data, wheel odometry measurements are used to predict initial pose for optimization. Required if visual-wheel setup used

+ `resize`

  Whether to downscale images by factor 2. NOTE: in order to use it, camera calibration intrinsic provided in YAML files have to be resized manually

+ `dilating_kernel_size`

  Kernel size for semantic masks dilation. Useful for avoiding dynamic points on object borders to be used in optimization

+ `roll_n`

  Roll planar constraint noise

+ `pitch_n`

  Pitch planar constraint noise

+ `zpw_n`

  Z-axis planar constraint noise. Decreasing can provide better reducing drift over vertical motion axis

+ `max_cnt`

  The number of features maintaned by feature extractor in an image stream

+ `min_dist`

  The minimal distance between extracted features on an image

+ `use_ransac`

  Use outlier rejection based on fundamental matrix and RANSAC. Useful for mono setups. In some cases, may diverge initialization performance

+ `F_threshold`

  Threshold for outlier rejection based on RANSAC (px)

+ `max_depth`

  Landmarks with larger depth values are filtered out

+ `flow_back`

  Use optical flow check for improving tracking results. May decrease runtime

+ `gnc`

  Use dynamic landmark rejection based on Graduated Non-Convexity. Additional sensor (IMU, wheel encoders) is required

+ `loss`

  Loss function used for dynamic landmarks rejection in optimization. Options: L1: 0; Geman-McClure: 1; Leclerc (recommended): 2

+ `fixed_shape`

  Constant shape parameter of a loss function. Larger values increase sensitivity to outliers

+ `convexity_init`

  Initial value of convexity. Has to be larger than `convexity_margin`. Larger values increases inference time of optimization, but increases outlier rejection performance

+ `convexity_margin`

  Margin convexity value to reach before finishing alternating optimization. Not recommended to change (set to 1 by default)

+ `convexity_update`

  Constant for updating shape of a loss function. Larger values decrease runtime, but lead to less stable optimization process. Not recommended to change (set to 1.4 by default)

+ `feature_weight_threshold`

  Landmarks with weigths less than given will not participate in optimization

+ `regularizer`

  Constant factor to regularize loss function. Not recommended to change (set to 2 by default)

+ `squared_eps`

  Multiplier for reprojection errors. Not recommended to change (set to 1 by default)

+ `max_solver_time`

  Optimization solver maximal time. Larger values lead to better accuracy but larger inference time

+ `max_num_iterations`

  Maximal solver inner interation number. Larger values lead to better accuracy but larger inference time

+ `keyframe_parallax`

  Parallax threshold for selecting keyframes (pixel). Smaller values lead to more keyframes, that increases runtime, but improves accuracy. Might deteriorate performance in dynamic environment

+ `acc_n`

  IMU accelerometer noise density

+ `gyr_n`

  IMU gyro noise density

+ `acc_w`

  IMU accelerometer random walk

+ `gyr_w`

  IMU gyro random walk

+ `estimate_td_inertial`

  Online calibration of timeshift between camera and IMU

+ `td_inertial`

  Prior timeshift value between camera and IMU
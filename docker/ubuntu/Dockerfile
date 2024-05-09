FROM nvidia/cuda:11.2.2-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DBUS_SESSION_BUS_ADDRESS=unix:path=/var/run/dbus/system_bus_socket

RUN mkdir -p /app/vins

COPY vins_env.yml /app/vins

WORKDIR /app/vins

RUN apt-get update && apt-get install -y apt-transport-https
RUN apt-get update && apt-get install build-essential software-properties-common -y \
    sudo \
    cmake \
    libboost-all-dev \
    graphviz \
    libgl1-mesa-dev \
    libglew-dev \
    libhdf5-dev \
    libfreetype6-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    sed \
    gcc-9 \
    g++-9 \
    zlib1g \
    ninja-build \
    wget \
    ffmpeg \
    libgflags-dev \
    python3 \
    python3-pip \
    git \
    unzip \
    python3-dev \
    libopenblas-dev \
    bzip2 \
    ca-certificates \
    curl \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libfmt-dev \
    libflann-dev

RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    /bin/bash miniconda.sh -b -p /opt/conda && \
    rm miniconda.sh

ENV PATH=$PATH:/opt/conda/bin

RUN conda env create -f vins_env.yml

SHELL ["conda", "run", "-n", "vins_env", "/bin/bash", "-c"]

ENV CONDA_DEFAULT_ENV=vins_env

RUN echo "conda activate vins_env" >> ~/.bashrc

RUN cd /app/vins && \
    git clone https://github.com/open-mmlab/mmcv.git && \
    cd mmcv && \
    pip install -r requirements/optional.txt && \
    pip install -v -e . && \
    cd ..

RUN cd /app/vins && \
    git clone -b main https://github.com/open-mmlab/mmsegmentation.git && \
    cd mmsegmentation && \
    pip install -v -e . && \
    cd ..

RUN cd /app/vins && \
    git clone -b 3.4.0 https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install -j4 && \
    cd ../.. && \
    rm -rf eigen

RUN cd /app/vins && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install -j4 && \
    cd ../.. && \
    rm -rf Sophus

RUN cd /app/vins && \
    git clone -b 1.14.x https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    mkdir ceres-bin && \
    cd ceres-bin && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_TESTING=OFF \
             -DBUILD_EXAMPLES=OFF && \
    make install -j4 && \
    cd ../.. && \
    rm -rf ceres-solver

RUN cd /app/vins && \
    git clone -b pcl-1.10.0 https://github.com/PointCloudLibrary/pcl && \
    cd pcl && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    echo "$(cat kdtree/CMakeFiles/pcl_kdtree.dir/link.txt) -llz4" > kdtree/CMakeFiles/pcl_kdtree.dir/link.txt && \
    make install -j4 && \
    cd ../.. && \
    rm -rf pcl

RUN cd /app/vins && \
    git clone -b 4.2.0 https://github.com/opencv/opencv.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DINSTALL_C_EXAMPLES=OFF \
             -DINSTALL_PYTHON_EXAMPLES=OFF && \
    make install -j4 && \
    cd ../.. && \
    rm -rf opencv

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install ros-noetic-desktop -y && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN apt-get update && apt-get install python3-rosdep -y \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    python3-catkin-tools && \ 
    rosdep init && \
    rosdep update

RUN apt-get update && apt-get install -y \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-rosbag \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros
   
CMD ["bash", "-c", "source activate env && /bin/bash"]

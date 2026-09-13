FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

# Some deployment networks (for example RTU MIREA UserGate) perform TLS
# inspection with a private CA. The build script can pass that CA as a
# BuildKit secret named "local_ca". It is installed into the image trust store
# before apt accesses HTTPS repositories. If no secret is supplied, the mount
# is simply absent and the normal public CA store is used.
RUN --mount=type=secret,id=local_ca,target=/usr/local/share/ca-certificates/rower-local-ca.crt,required=false \
    if [ -f /usr/local/share/ca-certificates/rower-local-ca.crt ]; then update-ca-certificates; fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       build-essential \
       cmake \
       git \
       libboost-dev \
       libeigen3-dev \
       python3-colcon-common-extensions \
       python3-rosdep \
       python3-serial \
       ros-jazzy-eigen3-cmake-module \
       ros-jazzy-navigation2 \
       ros-jazzy-nav2-bringup \
       ros-jazzy-slam-toolbox \
       ros-jazzy-robot-state-publisher \
       ros-jazzy-tf2-geometry-msgs \
       ros-jazzy-xacro \
       ros-jazzy-tf2-tools \
       ros-jazzy-teleop-twist-keyboard \
       ros-jazzy-rmw-cyclonedds-cpp \
       ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# RF2O has a ROS 2 branch but is not released as a Jazzy binary package for
# Ubuntu Noble/arm64. Build the upstream ROS 2 implementation from a pinned
# commit so Raspberry Pi builds are reproducible instead of depending on a
# non-existent ros-jazzy-rf2o-laser-odometry apt package.
#
# IMPORTANT for Raspberry Pi 4 / 4 GB: force a single compiler job. Eigen-heavy
# C++ compilation can otherwise start several cc1plus processes at once and push
# the Pi into swap/thrashing, making SSH appear frozen.
RUN mkdir -p /tmp/rf2o_ws/src/rf2o_laser_odometry \
    && cd /tmp/rf2o_ws/src/rf2o_laser_odometry \
    && git init \
    && git remote add origin https://github.com/MAPIRlab/rf2o_laser_odometry.git \
    && git fetch --depth 1 origin b38c68e46387b98845ecbfeb6660292f967a00d3 \
    && git checkout --detach FETCH_HEAD \
    && cd /tmp/rf2o_ws \
    && . /opt/ros/jazzy/setup.sh \
    && CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
         --executor sequential \
         --merge-install \
         --install-base /opt/rf2o \
         --packages-select rf2o_laser_odometry \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/rf2o_ws

WORKDIR /workspace/rower_gorod

CMD ["/bin/bash"]

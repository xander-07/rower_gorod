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
       python3-colcon-common-extensions \
       python3-rosdep \
       python3-serial \
       git \
       ros-jazzy-navigation2 \
       ros-jazzy-nav2-bringup \
       ros-jazzy-slam-toolbox \
       ros-jazzy-robot-state-publisher \
       ros-jazzy-xacro \
       ros-jazzy-tf2-tools \
       ros-jazzy-teleop-twist-keyboard \
       ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/rower_gorod

CMD ["/bin/bash"]

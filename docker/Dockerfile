# https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-core/Dockerfile
FROM ros:noetic-ros-core-focal as repo-deps
# This layer installs basic tools and the direct dependencies of vulkan_glasses_for_robots.

SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src/vulkan_glasses_for_robots


# Ensure package lists are clean
RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /etc/apt/sources.list.d/ros-latest.list
RUN rm -rf /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update --allow-insecure-repositories

# Install essential tools (again, to be sure)
RUN apt-get install -y --no-install-recommends curl gnupg --fix-missing

# Attempt to remove the problematic key (ignore errors)
RUN apt-key del F42ED6FBAB17C654 || true

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Update the package lists again
RUN apt-get update


WORKDIR /root/catkin_ws/

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \    
    &&  apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-noetic-catkin \
        python3-catkin-tools \
        python3-pip \        
        ros-noetic-tf-conversions \        
        python3-wstool  \
        libgoogle-glog-dev  \
        ros-noetic-mavros-msgs \
        libgflags-dev \
        libvulkan-dev \
        libglm-dev \
        ros-noetic-eigen-conversions \
        libboost-python-dev \
        python3-dev \
        nano \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && python3 -m pip install \
        wstool \
    && rm -rf /var/lib/apt/lists/*


FROM repo-deps as all-deps
# This layer installs dependencies of the other source packages.

COPY dependencies.rosinstall src/vulkan_glasses_for_robots/dependencies.rosinstall
RUN wstool init src src/vulkan_glasses_for_robots/dependencies.rosinstall
RUN wstool update -t src -j4

RUN apt-get update \
    && rosdep update \
    && source /opt/ros/noetic/setup.bash \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

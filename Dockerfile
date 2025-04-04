ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src



FROM osrf/ros:kinetic-desktop-full
ENV ROS_WS=/opt/catkin_ws

# install THORMANG dependencies packages
RUN mkdir -p /opt/catkin_ws/src && cd /opt/catkin_ws/src \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-OPC.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Common.git

# qt_build
# filters to keypoints

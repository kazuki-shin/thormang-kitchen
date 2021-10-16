FROM osrf/ros:kinetic-desktop-full
ENV ROS_WS=/opt/catkin_ws
ENV ROS_DISTRO=kinetic

RUN apt update && apt-get install -y \
    ros-kinetic-qt-build \
    ros-kinetic-map-server \
    ros-kinetic-nav-msgs \
    ros-kinetic-humanoid-nav-msgs \
    ros-kinetic-octomap \
    ros-kinetic-octomap-msgs \
    ros-kinetic-octomap-ros \
    ros-kinetic-octomap-server \
    ros-kinetic-qt-ros \
    ros-kinetic-sbpl \
    ssh \
    && apt-get clean

export ROS_MASTER_URI=http://10.17.3.35:11311
export ROS_HOSTNAME=10.17.3.30


# RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
#     && sed --in-place --expression \ 
#      '$isource "$ROS_WS/devel/setup.bash"' \
#       /ros_entrypoint.sh
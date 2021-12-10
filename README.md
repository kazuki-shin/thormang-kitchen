# THORMANG Kitchen

## Setup work enviornment on new machine

We use a ROS1 build environment. This project can easily be made to interface with ROS1 for visualization or for running on a real or simulated robot.
```
cd && mkdir -p catkin_ws/src && cd catkin_ws/src
```
```
git clone https://github.com/kazuki-shin/thormang-kitchen.git \
 && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Utility.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Tools.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Common.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-MPC.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-MPC-SENSORs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-PPC.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-OPC.git \
 && git clone https://github.com/ROBOTIS-GIT/humanoid_navigation.git 
```

```
sudo apt update && sudo apt-get install -y \
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
    ros-kinetic-urg-node \
    libv4l-dev \
    ros-kinetic-uvc-camera \
    madplay mpg321 \
    libncurses5-dev \
    && sudo apt-get clean
```

```
cd ~/catkin_ws && catkin_make && source devel/setup.bash 
```

## Running instructions (node launch order matters!)

set static ip
```
ip info
```
power on router client and connect to thormang wifi 



### 1. Perception PC
```
ssh robotis@10.17.3.35
pass: 111111
roscore
thormang sensor node
```
### 2. Motion PC 
```
ssh robotis@10.17.3.30
pass: 111111
thormang manager
```
### 3. Operating PC
```
Vision node
command node
```

clock synchronization 
add to bashrc
```
export ROS_MASTER_URI=http://10.17.3.35:11311
export ROS_HOSTNAME=<your IP>
```


## Notes
Change line 23 in humanoid_navigation/humanoid_localization/src/HumanoidLocalization.cpp
 from `include <pcl/filters/uniform_sampling.h>` to `include <pcl/keypoints/uniform_sampling.h>`

Useful tools
- rqt_graph
- rqt_plot


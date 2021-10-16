# THORMANG Kitchen

## How to Run

We use a ROS1 build environment. This project can easily be made to interface with ROS1 for visualization or for running on a real or simulated robot.
```
cd src
```
```
git clone https://github.com/kazuki-shin/thormang-kitchen.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-OPC.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-msgs.git \
 && git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Common.git \
 && git clone https://github.com/ROBOTIS-GIT/humanoid_navigation.git
```

1. Build Docker environment
```
docker build -t thormang-kitchen thormang-kitchen
```

2. Open a terminal inside the environment with this repository mounted
```
docker run -it -v `pwd`:/opt/catkin_ws/src thormang-kitchen

or 

thormang-kitchen/gui-docker -it -v `pwd`:/opt/catkin_ws/src thormang-kitchen
thormang-kitchen/gui-docker -c <container>
```

3. Go to `/opt/catkin_ws`, the mounted directory, build the package, and configure ROS
```

cd /opt/catkin_ws
catkin_make
source devel/setup.bash
```

4. Run the executable
```
rviz rviz
```

## Notes
Change line 23 in humanoid_navigation/humanoid_localization/src/HumanoidLocalization.cpp
 from `include <pcl/filters/uniform_sampling.h>` to `include <pcl/keypoints/uniform_sampling.h>`

## Commands
ssh -l robotis 10.17.3.30
ssh -l robotis 10.17.3.35
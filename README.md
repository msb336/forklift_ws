# forklift_ws
## Repo for all things TMHE at HMI in airsim
### Setup
You'll need:
[windows subsystem for linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
[ros kinetic or higher](https://janbernloehr.de/2017/06/10/ros-windows)
access to my fork of the [airsim_ros_pkgs](https://github.com/msb336/airsim_ros_pkgs) repo

**If on ros-kinetic:**
```
chmod +x setup.sh
./setup.sh
```

**Otherwise, you'll need to [set up your ros-workspace](wiki.ros.org/catkin) and clone the following dependencies into your source folder:**

[airsim_ros_pkgs](https://github.com/msb336/airsim_ros_pkgs)
[ros-navigation](https://github.com/ros-planning/navigation)
[navigation-msgs](https://github.com/ros-planning/navigation_msgs)
[tf2](https://github.com/ros/geometry2)


**and then build the following packages:**
``` 
catkin build airsim_ros_pkgs
catkin build costmap_2d
catkin build move_base
```


### Current State
**RL**
#### Goal
Assuming drone is some maximum distance and offset from pallet, determine proper control actions to align forks with pallet.

#### Status
- Input: 2d Distance transform of lidar data
- Reward Function: Angular and linear offset from desired line of approach
- ML library: CNTK
- Model: DQN

**ROS**
#### Goal
Move robot from some point A in the map to some goal point B provided by server (this will most likely be predetermined).
#### Status
Still pretty bare-bones. Listens to airsim sensor data and constructs a 2d costmap based on the robot pose and lidar output.
#### TODO
- Increase max object distance
- Integrate planner (use amcl, move_base, etc.)

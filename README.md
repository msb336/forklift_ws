# forklift_ws
## Repo for all things TMHE at HMI in airsim
### Setup
You'll need:
- [windows subsystem for linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
- [ros melodic](https://janbernloehr.de/2017/06/10/ros-windows)
- access to my fork of the [airsim_ros_pkgs](https://github.com/msb336/airsim_ros_pkgs) repo

make sure you are on the ros-melodic branch once you clone the repo

```
git clone https://github.com/msb336/forklift_ws.git -b ros-melodic
cd forklift_ws
chmod +x setup.sh
./setup.sh
```

**Otherwise, you'll need to [set up your ros-workspace](wiki.ros.org/catkin) and install the following dependencies:**

[airsim_ros_pkgs](https://github.com/msb336/airsim_ros_pkgs)
[ros-navigation](https://github.com/ros-planning/navigation)
[teb local planner]( https://github.com/rst-tu-dortmund/teb_local_planner)
[ackerman_msgs](https://github.com/ros-drivers/ackermann_msgs)


**and then build the following packages:**
``` 
catkin build airsim_ros_pkgs
catkin build ackermann_msgs
```

### To run the instance of ROS
You will need 3 terminals.
**In terminal 1:**
```
roscore
```
**In terminal 2:**
```
rviz
```
**In terminal 3:**
```
./run_car_demo.sh
```

or

```
source devel/setup.sh
roslaunch launch/warehousebot.launch
```


### Current State
**RL**
#### Goal
Assuming drone is some maximum distance and offset from pallet, determine proper control actions to align forks with pallet.

#### Status
- Input: 1d Distance transform of lidar data
- Reward Function: Linear offset from desired line of approach
- ML library: CNTK
- Model: DQN

**ROS**
#### Goal
Move robot from some point A in the map to some goal point B provided by server (this will most likely be predetermined).
#### Status
Control inputs need tuning. Car throttle maps to acceleration, so the throttle controller needs to be uned as well.
#### TODO
- Integrate dynamic obstacle avoidance

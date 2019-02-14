mkdir src; cd src
git clone https://github.com/msb336/airsim_ros_pkgs.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/ros-planning/navigation_msgs.git
git clone https://github.com/ros/geometry2.git
cd ..
catkin init
catkin build

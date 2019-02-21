mkdir src; cd src
git clone https://github.com/msb336/airsim_ros_pkgs.git
git clone https://github.com/ros-planning/navigation.git -b kinetic-devel
git clone https://github.com/ros-planning/navigation_msgs.git
git clone https://github.com/ros/geometry2.git -b kinetic-devel
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b kinetic-devel
git clone https://github.com/ros-drivers/ackermann_msgs.git
cd ..
catkin init
catkin config --extend /opt/ros/kinetic
catkin conif --merge-devel
catkin config --cmake-args -DCMAKE_CXX_FLAGS=-std=c++11
catkin build airsim_ros_pkg
catkin build costmap_2d
catkin build move_base

mkdir src; cd src
git clone https://github.com/msb336/airsim_ros_pkgs.git
git clone https://github.com/ros-drivers/ackermann_msgs.git

sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-teb-local-planner

cd ..
catkin init
catkin build airsim_ros_pkg
catkin build ackerman_msgs

sudo chmod +x /opt/ros/melodic/share/teb_local_planner/scripts/cmd_vel_to_ackermann_drive.py

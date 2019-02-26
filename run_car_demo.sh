source devel/setup.bash
rosparam delete move_base/global_costmap/footprint
rosparam delete move_base/local_costmap/footprint
rosparam delete costmap_2d/global_costmap/footprint
rosparam delete costmap_2d/local_costmap/footprint
roslaunch launch/warehousebot.launch

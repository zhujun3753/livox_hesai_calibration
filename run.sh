cd /home/zhujun/catkin_ws

catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"

# roslaunch fast_lio mapping_avia.launch
# rosbag play /home/zhujun/catkin_ws/src/FAST_LIO-main/data/2022-08-22-17-03-21.bag
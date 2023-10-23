#!/bin/bash
{
gnome-terminal -x bash -c "cd Examples/ROS2/;. install/local_setup.sh;ros2 run orbslam3 Mapsub 5 3 29 -25 48 -12 0.55 0.50 1 5;exec bash"
}&
sleep 2s
{
gnome-terminal -x bash -c "cd Examples/ROS2/;. install/local_setup.sh;ros2 run orbslam3 Mono-example_pub ~/ORB_SLAM3_ROS2_Map/Vocabulary/ORBvoc.txt ~/ORB_SLAM3_ROS2_Map/Examples/Monocular/KITTI00-02.yaml ~/ORB_SLAM3_ROS2_Map/KITTI/00;exec bash"
}

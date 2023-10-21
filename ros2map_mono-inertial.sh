#!/bin/bash
{
gnome-terminal -x bash -c "cd ~/usb_camera;. install/local_setup.sh;ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/fei/usb_camera/src/usb_cam/config/params.yaml;exec bash"
}&
sleep 2s
{
gnome-terminal -x bash -c "cd ~/wit_imu;. install/setup.sh;ros2 run ros2_imu imu_com;exec bash"
}&
sleep 2s
{
gnome-terminal -x bash -c "cd Examples/ROS2/;. install/local_setup.sh;ros2 run orbslam3 Mapsub 30 5 2 -2 2 -2 0.55 1 5;exec bash"
}&
sleep 2s
{
gnome-terminal -x bash -c "cd Examples/ROS2/;. install/local_setup.sh;ros2 run orbslam3 Mono-inertial_pub ~/ORB_SLAM3_ROS2_Map/Vocabulary/ORBvoc.txt ~/ORB_SLAM3_ROS2_Map/Examples/Monocular-Inertial/cam_imu.yaml;exec bash"
}

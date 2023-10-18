#!/bin/bash
{
gnome-terminal -x bash -c "cd ~/usb_camera;. install/local_setup.sh;ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/fei/usb_camera/src/usb_cam/config/params.yaml;exec bash"
}&
sleep 2s
{
gnome-terminal -x bash -c "cd Examples/ROS2/;. install/local_setup.sh;ros2 run orbslam3 mono ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples/Monocular/USBcam.yaml;exec bash"
}

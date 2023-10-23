## ros2编译
进入ORB_SLAM3_ROS2_Map
1)编译非ros
```
   ./build.sh
```
2)编译ros2
```
   ./build_ros2.sh
```
3)回到主目录，"ctrl+H",找到./bashrc文件，在文件最后添加
```
   export LD_LIBRARY_PATH=~/ORB_SLAM3_ROS2_Map/lib:$LD_LIBRARY_PATH
```
刷新
```
   source ~/.bashrc
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```


## ros2运行
## Run with rosbag
To play ros1 bag file, you should install `ros1 noetic` & `ros1 bridge`.  
Here is a [link](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/) to demonstrate example of `ros1-ros2 bridge` procedure.  
If you have `ros1 noetic` and `ros1 bridge` already, open your terminal and follow this:  
(Shell A, B, C, D is all different terminal, e.g. `stereo-inertial` mode)
1. Download EuRoC Dataset (`V1_02_medium.bag`)
```
$ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```  

2. Launch Terminal  
(e.g. `ROS1_INSTALL_PATH`=`/opt/ros/noetic`, `ROS2_INSTALL_PATH`=`/opt/ros/foxy`)
```
***Shell A:
source ${ROS1_INSTALL_PATH}/setup.bash
roscore

***Shell B:
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

***Shell C:
source ${ROS1_INSTALL_PATH}/setup.bash
rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

***Shell D:
source ${ROS2_INSTALL_PATH}/setup.bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]


## 设备运行(仅供参考)
相机:USB单目，1920x1080,mjpeg   IMU:wit HWT9073-CAN
##1.单目相机运行
  1)下载ros2版本usb_cam驱动源码
     [link](https://github.com/ros-drivers/usb_cam/tree/ros2)
  2)在主目录创建usb_camera/src，将下载的usb_cam驱动源码压缩包放入并解压重命名为usb_cam
  3)安装相关依赖
```
  rosdep install --from-paths src --ignore-src -y
```
    如果有依赖项目报错，请参考：
    [link](https://blog.csdn.net/liyuanjunfrank/article/details/122155262)
  4)修改usb_cam.cpp文件:删除所有计算时间的round()操作
  5)在usb_camera文件夹中进入终端
```
  colcon build
```
  6)根据自己的相机端口和分辨率修改usb_camera/src/usb_cam/config/params.yaml文件
  7)运行
```
    ./ros2_mono.sh
```

##2.单目+IMU运行
  1)根据维特智能提供的HWT9073-CAN通讯协议,编写ROS2通讯，我已经写好上传，直接下载编译
    [link](https://github.com/Ailce88899/wit_imu)
  2)连接相机和IMU，运行
```
    ./ros2_mono_inertial.sh
```


## ROS2-Grid-Mapping运行
## 1.数据集运行
   下载Kitti数据集:
   [link](https://pan.baidu.com/s/1l8DdCrvx0990vyYvEGJUkg?pwd=c2uf) 提取码: c2uf 
   解压并放至ORB_SLAM3_ROS2_Map文件夹中
```
   ./ros2map_mono_kitti-example.sh
```

## 2.单目相机
  连接设备,运行
```
  ./ros2map_mono.sh
```
## 3.单目相机+IMU
  连接设备，运行
```
  ./ros2map_mono-inertial.sh
```
  

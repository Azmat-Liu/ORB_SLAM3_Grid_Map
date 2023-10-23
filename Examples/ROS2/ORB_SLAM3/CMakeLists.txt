cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV 4.2 QUIET)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL 1.10 REQUIRED)

include_directories(
 /usr/include
)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(mono
  src/monocular/ros2_mono.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(mono-inertial
  src/monocular-inertial/ros2_mono_inertial.cpp
)
ament_target_dependencies(mono-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

#Node for Grid Mapping
add_executable(Monopub
  src/grid-map/ros2_mono_pub.cpp
)
ament_target_dependencies(Monopub rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(Monopub
  ${PCL_LIBRARIES}
)

add_executable(Mono-inertial_pub
  src/grid-map/ros2_mono_inertial_pub.cpp
)
ament_target_dependencies(Mono-inertial_pub rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(Mono-inertial_pub 
  ${PCL_LIBRARIES}
)

add_executable(Mono-example_pub
  src/grid-map/ros2_mono_data-example_pub.cpp
)
ament_target_dependencies(Mono-example_pub rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(Mono-example_pub
  ${PCL_LIBRARIES}
)

add_executable(Mapsub
  src/grid-map/grid_map_sub.cpp
)
ament_target_dependencies(Mapsub rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(Mapsub
  ${PCL_LIBRARIES}
)


install(TARGETS 
  mono 
  mono-inertial
  rgbd 
  stereo 
  stereo-inertial
  Monopub
  Mono-inertial_pub
  Mono-example_pub
  Mapsub
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()


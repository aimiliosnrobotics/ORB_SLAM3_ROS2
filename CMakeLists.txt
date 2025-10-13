cmake_minimum_required(VERSION 3.5)
project(orbslam3)
add_definitions(-w)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages")

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
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
# find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

# Define ORB_SLAM3 paths for STEREO-FIXED version
set(ORB_SLAM3_ROOT_DIR "/home/ubuntu/ORB-SLAM3-STEREO-FIXED")
set(ORB_SLAM3_LIBRARY "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/lib/libORB_SLAM3.so")
set(ORB_SLAM3_INCLUDE_DIR "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/include")
set(DBoW2_LIBRARY "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/Thirdparty/DBoW2/lib/libDBoW2.so")
set(DBoW2_INCLUDE_DIR "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/Thirdparty/DBoW2")
set(g2o_LIBRARY "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/Thirdparty/g2o/lib/libg2o.so")
set(g2o_INCLUDE_DIR "/home/ubuntu/ORB-SLAM3-STEREO-FIXED/Thirdparty/g2o")

# Add explicit include paths for ROS2 headers and ORB_SLAM3 dependencies
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
  /opt/ros/jazzy/include/cv_bridge/cv_bridge
  /opt/ros/jazzy/include/tf2_geometry_msgs/tf2_geometry_msgs
)

link_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs cv_bridge Pangolin OpenCV)
target_link_libraries(mono ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(mono-pcloud
  src/mono-pcloud/mono-pcloud.cpp
  src/mono-pcloud/mono-pcloud-node.cpp
)
ament_target_dependencies(mono-pcloud rclcpp sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs image_transport cv_bridge Pangolin OpenCV)
target_link_libraries(mono-pcloud ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(mono-inetrial
  src/monocular-inetrial/mono-inetrial.cpp
  src/monocular-inetrial/mono-inetrial-node.cpp
)
ament_target_dependencies(mono-inetrial rclcpp sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs cv_bridge Pangolin OpenCV)
target_link_libraries(mono-inetrial ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(mono-stream
  src/monocular-stream/mono-stream.cpp
  src/monocular-stream/mono-stream-node.cpp
)
ament_target_dependencies(mono-stream rclcpp sensor_msgs cv_bridge Pangolin OpenCV)
target_link_libraries(mono-stream ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(image-stream
  src/image-stream/image-stream.cpp
  src/image-stream/image-stream-node.cpp
)
ament_target_dependencies(image-stream rclcpp sensor_msgs camera_info_manager cv_bridge OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters Pangolin OpenCV)
target_link_libraries(rgbd ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters Pangolin OpenCV)
target_link_libraries(stereo ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros tf2 tf2_geometry_msgs cv_bridge message_filters Pangolin OpenCV)
target_link_libraries(stereo-inertial ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})

install(TARGETS mono mono-pcloud mono-inetrial mono-stream image-stream rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/imu_frame_corrector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
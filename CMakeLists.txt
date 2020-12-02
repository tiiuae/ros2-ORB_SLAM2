cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

set(ORB_SLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)
set(ORB_SLAM ${ORB_SLAM_DIR}/lib/libORB_SLAM3.so)
set(G2O ${ORB_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so)
set(DBOW2 ${ORB_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)

set($ENV{ORB_SLAM2_INCLUDE_DIR} "${ORB_SLAM_DIR}/include/")

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
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${ORB_SLAM_DIR}
  ${ORB_SLAM_DIR}/include/
  ${ORB_SLAM_DIR}/include/CameraModels/
)

add_custom_command(OUTPUT ${ORB_SLAM} ${G2O} ${DBOW2}
  WORKING_DIRECTORY ${ORB_SLAM_DIR}
  COMMAND sh build.sh
)

add_custom_target(
    orb_slam
    DEPENDS ${ORB_SLAM}
)

add_executable(mono 
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

add_dependencies(mono orb_slam)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge Pangolin)
target_link_libraries(mono ${ORB_SLAM} ${G2O})

add_executable(rgbd 
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)

add_dependencies(rgbd orb_slam)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters Pangolin)
target_link_libraries(rgbd ${ORB_SLAM} ${G2O})

add_executable(stereo 
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)

add_dependencies(stereo orb_slam)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters Pangolin)
target_link_libraries(stereo ${ORB_SLAM} ${G2O})

install(TARGETS mono rgbd stereo
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${ORB_SLAM} ${G2O} ${DBOW2}
  DESTINATION lib)

install(
  FILES
    src/stereo/depthai_stereo_orbs2.yaml
    ${ORB_SLAM_DIR}/Vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME})

ament_package()

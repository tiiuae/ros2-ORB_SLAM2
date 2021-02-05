cmake_minimum_required(VERSION 3.5)
project(orbslam_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

add_subdirectory(ORB_SLAM3)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ORB_SLAM3
        ORB_SLAM3/include
        ORB_SLAM3/include/CameraModels
)


add_executable(ros_mono
        src/ros_mono.cc
        )

#add_executable(ros_mono_inertial
#        src/ros_mono_inertial.cc
#        )

ament_target_dependencies(ros_mono rclcpp sensor_msgs cv_bridge)
target_link_libraries(ros_mono ORB_SLAM3 )

#ament_target_dependencies(ros_mono_inertial rclcpp sensor_msgs cv_bridge)
#target_link_libraries(ros_mono_inertial ORB_SLAM3_LIBRARIES)

install(TARGETS ros_mono DESTINATION lib/${PROJECT_NAME} )
install(TARGETS ORB_SLAM3 DESTINATION lib/${PROJECT_NAME} )
install(TARGETS ORB_SLAM3 DESTINATION lib/${PROJECT_NAME} )


install(DIRECTORY ORB_SLAM3/Vocabulary
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

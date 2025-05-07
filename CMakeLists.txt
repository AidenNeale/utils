cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

##########
add_library(utils SHARED src/utils/utils.cpp)
ament_target_dependencies(utils
  "laser_geometry"
  "pcl_ros"
)
install(TARGETS
  utils
  DESTINATION lib/${PROJECT_NAME}
)
#########
add_library(ros2bag_reader SHARED 
  src/utils/ros2bag_reader.cpp
)
ament_target_dependencies(ros2bag_reader
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "rosbag2_transport"
)
install(TARGETS
  ros2bag_reader
  DESTINATION lib/${PROJECT_NAME}
)
#########
add_executable(rosbag_to_video 
  src/utils/video_from_rosbag.cpp
)
ament_target_dependencies(rosbag_to_video
  cv_bridge
  OpenCV
)
target_link_libraries(rosbag_to_video
  ros2bag_reader
  yaml-cpp::yaml-cpp
  ${OpenCV_LIBS}
)

install(TARGETS
  rosbag_to_video 
  DESTINATION lib/${PROJECT_NAME}
)
###############
add_executable(pointcloud_from_laserscans 
  src/utils/pointcloud_from_laserscans.cpp
)
ament_target_dependencies(pointcloud_from_laserscans
  rclcpp
  sensor_msgs
  laser_geometry
  rosbag2_cpp
  rosbag2_storage
  pcl_ros
)
target_link_libraries(pointcloud_from_laserscans
  ros2bag_reader
  yaml-cpp::yaml-cpp
  utils
)
install(TARGETS
  pointcloud_from_laserscans 
  DESTINATION lib/${PROJECT_NAME}
)
###############
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
##############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

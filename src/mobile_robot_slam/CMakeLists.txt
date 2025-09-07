cmake_minimum_required(VERSION 3.15)
project(mobile_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pcl_processor src/pcl_processor.cpp)
ament_target_dependencies(pcl_processor 
  rclcpp 
  sensor_msgs 
  PCL 
  pcl_ros 
  pcl_conversions
  Eigen3
)

target_link_libraries(pcl_processor ${PCL_LIBRARIES})
target_include_directories(pcl_processor PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_processor PRIVATE ${PCL_DEFINITIONS})

# Install C++ executable
install(TARGETS
  pcl_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

cmake_minimum_required(VERSION 3.8)
project(agro_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rcl_interfaces
)

include_directories(include)

add_executable(sim_movement_controller src/sim_movement_controller.cpp)
# add_executable(imu src/imu.cpp)
ament_target_dependencies(sim_movement_controller rclcpp sensor_msgs std_msgs geometry_msgs)

install(TARGETS
 sim_movement_controller
 DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/gz/controller.py
  scripts/keyboard_controller.py
  scripts/camera_driver.py
  scripts/cloud_to_scan.py
  scripts/image_to_detection.py
  scripts/path_mapping.py
  scripts/path_controller.py
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/util/utils.py
  scripts/util/astar.py
  scripts/util/hybrid_astar.py
  scripts/util/parallel.py
  DESTINATION lib/${PROJECT_NAME}/util
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}/
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

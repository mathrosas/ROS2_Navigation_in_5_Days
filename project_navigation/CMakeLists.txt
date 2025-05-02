cmake_minimum_required(VERSION 3.8)
project(project_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(project_localization REQUIRED)
find_package(project_mapping REQUIRED)
find_package(project_path_planning REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

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

add_executable(spot_recorder src/spot_recorder.cpp)
ament_target_dependencies(
  spot_recorder
  rclcpp
  geometry_msgs
)

add_executable(move_to_spot src/move_to_spot.cpp)
ament_target_dependencies(
  move_to_spot
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

install(TARGETS
  spot_recorder
  move_to_spot
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(project_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(project_mapping REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyServiceMessage.srv"
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

add_executable(spot_recorder src/spots_to_file.cpp)
ament_target_dependencies(spot_recorder rclcpp geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(spot_recorder ${cpp_typesupport_target})

install(TARGETS
   spot_recorder
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

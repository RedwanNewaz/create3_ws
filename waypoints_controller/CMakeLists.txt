cmake_minimum_required(VERSION 3.8)
project(waypoints_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(waypoints_tracker REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(waypoints_controller SHARED 
  src/waypoints_controller.h
  src/waypoints_controller.cpp
  src/rapidcsv.h
  )
target_include_directories(waypoints_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(waypoints_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  waypoints_controller
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp_action"
  "rclcpp_components"
  "waypoints_tracker"
)


rclcpp_components_register_node(waypoints_controller PLUGIN "navigation::WaypointController" EXECUTABLE waypoints_action_server)
install(TARGETS 
  waypoints_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

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

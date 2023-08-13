cmake_minimum_required(VERSION 3.8)
project(create3_map_server)

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
find_package(rcl_interfaces REQUIRED)
find_package(waypoints_tracker REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(create3_map_server SHARED
  src/create3_map_server.h
  src/create3_map_server.cpp)
target_link_libraries(create3_map_server ${OpenCV_LIBRARIES})
target_include_directories(create3_map_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(create3_map_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  create3_map_server
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp_action"
  "rclcpp_components"
  "rcl_interfaces"
  "waypoints_tracker"
)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "map_server::DynamicMap" EXECUTABLE dynamic_map_server)

install(TARGETS
${PROJECT_NAME}
			ARCHIVE DESTINATION lib
			LIBRARY DESTINATION lib
			RUNTIME DESTINATION bin
			)
	ament_export_dependencies(rosidl_default_runtime )

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

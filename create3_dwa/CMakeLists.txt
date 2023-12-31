cmake_minimum_required(VERSION 3.8)
project(create3_dwa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp 0.8.0 REQUIRED)

add_library(create3_dwa SHARED
  src/create3_dwa.cpp src/dwa_planner.cpp)
target_include_directories(create3_dwa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(create3_dwa PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(create3_dwa Eigen3::Eigen yaml-cpp::yaml-cpp)

ament_target_dependencies(
  create3_dwa
  "rclcpp"
  "rclcpp_components"
  "visualization_msgs"
  "irobot_create_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2"
)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "airlab::create3_dwa" EXECUTABLE ${PROJECT_NAME}_node)
install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
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
ament_export_dependencies(rosidl_default_runtime)
ament_package()

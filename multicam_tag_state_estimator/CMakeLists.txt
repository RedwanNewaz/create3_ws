cmake_minimum_required(VERSION 3.8)
project(multicam_tag_state_estimator)

set(CXX_STANDARD 17)
include(cmake/CPM.cmake)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(multicam_tag_state_estimator src/multicam_tag_state_estimator.cpp)
target_include_directories(multicam_tag_state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multicam_tag_state_estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  multicam_tag_state_estimator
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "rclcpp_components"
  "rcl_interfaces"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "cv_bridge"
)
find_package(OpenCV REQUIRED)
find_package(apriltag 3.3.0 REQUIRED)
find_package(yaml-cpp 0.8.0 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} apriltag ${OpenCV_LIBRARIES} Eigen3::Eigen yaml-cpp::yaml-cpp) 


install(TARGETS multicam_tag_state_estimator
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()

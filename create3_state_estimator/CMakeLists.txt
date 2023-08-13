cmake_minimum_required(VERSION 3.8)
project(create3_state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(ThirdParty/sensor_fusion)

include_directories(
  ThirdParty/sensor_fusion/include  
  include/create3_state_estimator)
set(HEADER_FILES 
  include/create3_state_estimator/ComplementaryFilter.h
  include/create3_state_estimator/EKF.h
  include/create3_state_estimator/Pose.h
  include/create3_state_estimator/StateMachine.h 
  include/create3_state_estimator/StateEstimatorBase.h
  include/create3_state_estimator/JointStateEstimator.h
  include/create3_state_estimator/LoggerCSV.h
  include/create3_state_estimator/FilterBase.h
  )

add_executable(create3_state_estimator ${HEADER_FILES}
  src/EKF.cpp
  src/JointStateEstimator.cpp
  src/create3_state_estimator.cpp)
target_link_libraries(create3_state_estimator Eigen3::Eigen sensor_fusion)

target_include_directories(create3_state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(create3_state_estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  create3_state_estimator
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "rclcpp_action"
  "rclcpp_components"
  "rcl_interfaces"
  "tf2"
  "tf2_ros"
  "irobot_create_msgs"
)

install(TARGETS create3_state_estimator
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

cmake_minimum_required(VERSION 3.8)
project(lerobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_core REQUIRED)
find_package(robot_sim REQUIRED)
find_package(feetech_cpp_lib REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lerobot_sim src/lerobot_sim.cpp)
ament_target_dependencies(lerobot_sim
    robot_sim Eigen3)
target_include_directories(lerobot_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lerobot>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lerobot_sim PUBLIC c_std_99 cxx_std_17)

add_executable(lerobot_hw src/lerobot_hw.cpp)
ament_target_dependencies(lerobot_hw
    robot_core feetech_cpp_lib Eigen3)
target_include_directories(lerobot_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lerobot>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lerobot_hw PUBLIC c_std_99 cxx_std_17)

add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher
    rclcpp geometry_msgs visualization_msgs sensor_msgs nav_msgs tf2_ros)
target_include_directories(path_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lerobot>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


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

install(TARGETS lerobot_sim lerobot_hw path_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  rviz 
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

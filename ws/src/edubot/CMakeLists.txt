cmake_minimum_required(VERSION 3.8)
project(edubot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(common REQUIRED)
#find_package(kinematics REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Add include directories
include_directories($<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

add_executable(robot_hw src/robot_hw.cpp src/robot.cpp)
ament_target_dependencies(robot_hw 
    rclcpp Eigen3 common sensor_msgs trajectory_msgs)
target_include_directories(robot_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/edubot>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_hw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(robot_sim src/robot_sim.cpp src/robot.cpp)
ament_target_dependencies(robot_sim
    rclcpp Eigen3 common sensor_msgs trajectory_msgs)
target_include_directories(robot_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/edubot>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#add_executable(path_publisher src/path_publisher.cpp)
#ament_target_dependencies(path_publisher
#    rclcpp geometry_msgs visualization_msgs sensor_msgs nav_msgs tf2_ros kinematics)
#target_include_directories(path_publisher PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/edubot>
#  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(path_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS robot_hw robot_sim #path_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  rviz 
  urdf
  meshes
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

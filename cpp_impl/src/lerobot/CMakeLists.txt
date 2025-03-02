cmake_minimum_required(VERSION 3.8)
project(lerobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lerobot_sim src/lerobot_sim.cpp)
ament_target_dependencies(lerobot_sim
    robot Eigen3)
target_include_directories(lerobot_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lerobot>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lerobot_sim PUBLIC c_std_99 cxx_std_17)


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

install(TARGETS lerobot_sim #path_publisher
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

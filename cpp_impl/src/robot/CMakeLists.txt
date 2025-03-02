cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
 
# define a library target called robot
add_library(robot src/robot.cpp)
ament_target_dependencies(robot 
    rclcpp sensor_msgs trajectory_msgs)
target_include_directories(robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot>
  $<INSTALL_INTERFACE:include>)

# add this line to exports the library
ament_export_targets(robot HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/robot
  DESTINATION include
)
 
install(
  TARGETS robot
  EXPORT robot
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs trajectory_msgs)
ament_export_libraries(robot)

ament_package()

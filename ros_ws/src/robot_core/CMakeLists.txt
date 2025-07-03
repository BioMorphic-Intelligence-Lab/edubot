cmake_minimum_required(VERSION 3.8)
project(robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  DEPENDENCIES builtin_interfaces
)
 
# define a library target called robot
add_library(robot src/robot.cpp)
ament_target_dependencies(robot
    rclcpp sensor_msgs trajectory_msgs)
# Ensure message headers are available
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot ${cpp_typesupport_target})

target_include_directories(robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_core>
  $<INSTALL_INTERFACE:include>)

# add this line to exports the library
ament_export_targets(robot HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/robot_core
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

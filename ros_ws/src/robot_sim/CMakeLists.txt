cmake_minimum_required(VERSION 3.8)
project(robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_core REQUIRED)
find_package(rclcpp REQUIRED)


add_library(robot_sim src/robot_sim.cpp)
ament_target_dependencies(robot_sim
    rclcpp robot_core)
target_link_libraries(robot_sim ${cpp_typesupport_target})

target_include_directories(robot_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_sim>
  $<INSTALL_INTERFACE:include>)

# add this line to exports the library
ament_export_targets(robot_sim HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/robot_sim
  DESTINATION include
)
 
install(
  TARGETS robot_sim
  EXPORT robot_sim
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp robot_core)
ament_export_libraries(robot_sim)

ament_package()
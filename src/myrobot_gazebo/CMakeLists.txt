cmake_minimum_required(VERSION 3.8)
project(myrobot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(myrobot_grasp_attach SHARED src/grasp_attach_plugin.cpp)
target_include_directories(myrobot_grasp_attach PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(myrobot_grasp_attach ${GAZEBO_LIBRARIES})
ament_target_dependencies(myrobot_grasp_attach gazebo_ros rclcpp std_srvs)

install(TARGETS myrobot_grasp_attach
  LIBRARY DESTINATION lib)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(gazebo_ros)

ament_package()

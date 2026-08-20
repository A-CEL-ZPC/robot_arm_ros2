cmake_minimum_required(VERSION 3.8)
project(myrobot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 导出编译命令（compile_commands.json），供 VSCode/clangd 代码提示使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
# 可执行文件
add_executable(test_moveit src/test_moveit.cpp)
add_executable(commander_template src/commander_template.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)
ament_target_dependencies(commander_template rclcpp moveit_ros_planning_interface example_interfaces)
install(TARGETS test_moveit
  commander_template
  DESTINATION lib/${PROJECT_NAME})

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

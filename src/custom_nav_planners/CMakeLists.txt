cmake_minimum_required(VERSION 3.8)
project(custom_nav_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)
 
add_library(custom_nav_planners SHARED
  src/custom_global_planner.cpp
  src/custom_local_planner.cpp
)
 
ament_target_dependencies(custom_nav_planners
  rclcpp
  nav2_core
  nav2_msgs
  geometry_msgs
  pluginlib
)
 
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
 
install(TARGETS custom_nav_planners
  DESTINATION lib
)
 
install(DIRECTORY include/
  DESTINATION include/
)
 
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 正确安装行为树文件到 package:// 路径可识别的位置
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees/
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

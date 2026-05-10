cmake_minimum_required(VERSION 3.22)
project(dofbot_pro_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)


add_executable(set_target_joints src/set_target_joints.cpp)
ament_target_dependencies(set_target_joints rclcpp moveit_ros_planning_interface graph_msgs moveit_visual_tools)

add_executable(set_target_position src/set_target_position.cpp)
ament_target_dependencies(set_target_position rclcpp moveit_ros_planning_interface)

add_executable(random_move src/random_move.cpp)
ament_target_dependencies(random_move rclcpp moveit_ros_planning_interface)

add_executable(cartesian_path src/cartesian_path.cpp)
ament_target_dependencies(cartesian_path rclcpp moveit_ros_planning_interface graph_msgs moveit_visual_tools) 

add_executable(multi_track_motion src/multi_track_motion.cpp)
ament_target_dependencies(multi_track_motion rclcpp moveit_ros_planning_interface graph_msgs moveit_visual_tools) 

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp moveit_ros_planning_interface graph_msgs moveit_visual_tools moveit_msgs shape_msgs)  

add_executable(set_scene src/set_scene.cpp)
ament_target_dependencies(set_scene rclcpp moveit_ros_planning_interface graph_msgs moveit_visual_tools moveit_msgs shape_msgs)

install(TARGETS
  set_target_joints
  set_target_position
  cartesian_path
  multi_track_motion
  random_move
  obstacle_avoidance
  set_scene
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

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

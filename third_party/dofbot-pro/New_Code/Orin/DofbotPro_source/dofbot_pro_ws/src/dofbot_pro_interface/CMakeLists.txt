cmake_minimum_required(VERSION 3.8)
project(dofbot_pro_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Kinemarics.srv"
  "srv/CurJoint.srv"
  "srv/DofbotProKinemarics.srv"
  "msg/JointInfo.msg"
  "msg/PosInfo.msg"
  "msg/Position.msg"
  "msg/ArmJoint.msg"
  "msg/ImageMsg.msg"
  "msg/WidthInfo.msg"
  "msg/AprilTagInfo.msg"
  "msg/Yolov11Detect.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 包的包含目录
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rosmsg_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS}
  ${moveit_commander_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
)

# 安装消息和服务文件
install(DIRECTORY msg srv
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

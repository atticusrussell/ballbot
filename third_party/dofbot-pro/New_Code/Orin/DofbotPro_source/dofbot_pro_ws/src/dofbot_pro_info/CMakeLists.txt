cmake_minimum_required(VERSION 3.8)
project(dofbot_pro_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(dofbot_pro_interface REQUIRED)



## **生成可执行文件**
add_executable(kinemarics_dofbot src/kinemarics_dofbotpro.cpp)


## **修正 include 目录**
target_include_directories(kinemarics_dofbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

## **设置 C++ 标准**
target_compile_features(kinemarics_dofbot PUBLIC c_std_99 cxx_std_17)



## **设定库依赖**
ament_target_dependencies(
  kinemarics_dofbot
  rclcpp
  kdl_parser
  std_msgs
  dofbot_pro_interface
)

## **链接外部库**
target_link_libraries(kinemarics_dofbot
  /usr/lib/libdofbotpro_kinemarics.so
)



## **安装共享库**
#install(TARGETS dofbotpro_kinemarics
 # EXPORT export_${PROJECT_NAME}
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
#)

## **安装可执行文件**
install(TARGETS kinemarics_dofbot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

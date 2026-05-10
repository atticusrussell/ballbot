cmake_minimum_required(VERSION 3.5)
project(ballbot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED) # Not strictly needed unless you use urdf_check

install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

# --- Final Package Setup ---
ament_package()

cmake_minimum_required(VERSION 3.8)
project(gp12_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF/Xacro files
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes
install(DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
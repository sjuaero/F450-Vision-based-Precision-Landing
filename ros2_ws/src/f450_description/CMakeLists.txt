cmake_minimum_required(VERSION 3.8)
project(f450_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch meshes worlds models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

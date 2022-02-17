cmake_minimum_required(VERSION 3.5)

project(ur_example_dual_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY etc launch resources scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()

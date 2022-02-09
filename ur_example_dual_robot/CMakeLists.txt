cmake_minimum_required(VERSION 3.0.2)
project(ur_example_dual_robot)

find_package(catkin REQUIRED )

catkin_package(
)

install(FILES etc/alice_controllers.yaml etc/bob_controllers.yaml etc/view_robot.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)
install(FILES
  launch/dual_robot_startup.launch
  launch/load_dual_description.launch
  launch/view_dual_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  resources/ursim/programs_alice/default.installation
  resources/ursim/programs_alice/default.variables
  resources/ursim/programs_alice/external_control.urp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/programs_alice
)
install(FILES
  resources/ursim/programs_bob/default.installation
  resources/ursim/programs_bob/default.variables
  resources/ursim/programs_bob/external_control.urp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/programs_bob
)
install(FILES
  resources/ursim/urcaps/externalcontrol-1.0.5.jar
  resources/ursim/urcaps/ros_as_a_service-0.1-beta4.jar
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources/ursim/urcaps
)
install(FILES scripts/docker_alice_bob.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES urdf/dual_robot.xacro
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

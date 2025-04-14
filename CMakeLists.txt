cmake_minimum_required(VERSION 3.0.2)
project(hydra_tic_tac_toe)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cartesian_planning_server
  pyrobopath_ros
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  cartesian_planning_server
  pyrobopath_ros
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/tic_tac_toe_exec.py
  scripts/visualize_collision_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

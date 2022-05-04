cmake_minimum_required(VERSION 3.0.2)
project(py_joint_state_pub)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/gazebo_joint_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

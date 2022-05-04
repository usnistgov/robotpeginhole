cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugins2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set (PLUGIN_DESTINATION  "C:/opt/ros/noetic/gzpegboard/pluginswin")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
)

add_service_files(DIRECTORY srv FILES
  GetJointsProperties.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  )


find_path(Boost_INCLUDEDIR C:/opt/ros/noetic/x64/include)
find_path(Boost_LIBRARYDIR C:/opt/ros/noetic/x64/Lib)
find_path(GAZEBO_INCLUDEDIR C:/opt/ros/noetic/x64/include/gazebo-10)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS thread program_options iostreams date_time)
find_package(Protobuf REQUIRED)  
find_package(OGRE REQUIRED CONFIG)

# The gazebo headache
#find_package(gazebo)
link_directories(${GAZEBO_LIBRARY_DIRS} )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DWIN32 ${CMAKE_CXX_FLAGS}")
    # turn off various warnings - none needed in this compile
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4068")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4100")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4101")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4127") 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4244")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4267")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4273")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4275")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4305") 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4458")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_BIND_GLOBAL_PLACEHOLDERS")

    # THis enables pdb symbol debug file. Not real important for plugins.
    set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " /DEBUG")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gripper_plugin
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Added Define for Windows
set(CMAKE_CXX_FLAGS "-D_WIN32_WINNT=0x0601 ${CMAKE_CXX_FLAGS}")


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  C:/opt/ros/noetic/x64/include/boost
  C:/opt/ros/noetic/x64/include/gazebo-10
  C:/opt/ros/noetic/x64/include/ignition/math4
  C:/opt/ros/noetic/x64/include/ignition/transport4
  C:/opt/ros/noetic/x64/include/ignition/msgs1
  C:/opt/ros/noetic/x64/include/sdformat-6.2
  C:/opt/ros/noetic/gztaskboard/install/include
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/gripper_plugin_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(C:/opt/ros/noetic/x64/Lib)

## Declare a C++ library
add_library(gazebo_ros_joint_state_publisher 
   src/gazebo_ros_joint_state_publisher.cpp
 )

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_ros_joint_state_publisher
   ${catkin_LIBRARIES}
   C:/opt/ros/noetic/x64/Lib/tbb.lib
   C:/opt/ros/noetic/x64/Lib/libprotobuf.lib
   ${GAZEBO_LIBRARIES} 
   ${Boost_LIBRARIES}

    C:/opt/ros/noetic/x64/Lib/gazebo_transport.lib
    C:/opt/ros/noetic/x64/Lib/gazebo.lib
    C:/opt/ros/noetic/x64/Lib/gazebo_common.lib
    C:/opt/ros/noetic/x64/Lib/gazebo_msgs.lib
    C:/opt/ros/noetic/x64/Lib/sdformat.lib
    C:/opt/ros/noetic/x64/Lib/ignition-transport4.lib
    C:/opt/ros/noetic/x64/Lib/ignition-math4.lib
    C:/opt/ros/noetic/x64/Lib/gazebo_physics.lib
    C:/opt/ros/noetic/x64/Lib/boost_thread-vc140-mt.lib
  
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS gazebo_ros_joint_state_publisher
   RUNTIME DESTINATION ${PLUGIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gripper_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

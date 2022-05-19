
#pragma once

// C++ std lib
#include <thread>
#include <cstdarg>
#include <iostream>
#include <fstream>


// Boost
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>

// Error at compile time for non handled convert
#include <boost/static_assert.hpp>

// ROS messages
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>

// my ROS workspace headers
#include "crpi_gazebo.h"
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"

// Eigen include
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <gazebo_plugins2/GetJointsProperties.h>
#include <gazebo_plugins2/GuardedMove.h>
#include <test_gazebo_package/conversions.h>



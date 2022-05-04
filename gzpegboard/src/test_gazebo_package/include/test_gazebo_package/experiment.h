#ifndef EXPERIMENTS_H
#define EXPERIMENTS_H

#include "test_gazebo_package/TestGazeboRobot.h"

extern void testConversions(); // Test conversions
extern void testcurrentRobotJoints();
extern void testcurrentPose();
extern void acquirePeg(std::string pegname);
extern void insertPegDeadReckoning(std::string holename);
extern void approachPegHole(std::string holename);
extern void guardedMovePegArray(std::string holename,double ft_threahold, tf::Vector3 fudge, size_t updaterate=10);

#endif
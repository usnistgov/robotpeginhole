#ifndef TESTGAZEBOROBOT
#define TESTGAZEBOROBOT

#include <vector>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"
#include "test_gazebo_package/wm.h"
/**
     * @brief getCmdOption simple commmand line option. Slow. But simple.
     * https://gist.github.com/plasticbox/3708a6cdfbece8cd224487f9ca9794cd
     * Example: std::string ip = getCmdOption(argc, argv, "-ip=");
     * @param argc
     * @param argv
     * @param option
     * @return
     */
inline std::string getCmdOption(std::vector<std::string> args, const std::string &option, std::string szDefault = "")
{
    std::string cmd;
    for (int i = 0; i < args.size(); ++i)
    {
        std::string arg = args[i];
        if (0 == arg.find(option))
        {
            //std::size_t found = arg.find(option);
            cmd = arg.substr(option.size());
            boost::algorithm::trim(cmd);
            return cmd;
        }
    }
    return szDefault;
}



inline geometry_msgs::Point geostore(double a, double b, double c)
{
    geometry_msgs::Point p1;
    p1.x = a;
    p1.y = b;
    p1.z = c;
    return p1;
}

inline void trajconvert(moveit_msgs::RobotTrajectory intraj,
                        std::vector<geometry_msgs::Pose> &pose_traj,
                        std::vector<trajectory_msgs::JointTrajectoryPoint> &jt_traj)
{
    for (size_t i = 0; i < intraj.joint_trajectory.points.size(); i++)
    {
        // assumes robot coordinate system
        trajectory_msgs::JointTrajectoryPoint pt = intraj.joint_trajectory.points[i];
        jt_traj.push_back(pt);

        geometry_msgs::Pose pose = conversion::Convert<tf::Pose, geometry_msgs::Pose>(
            robot._arm->FK(pt.positions));
        //          robot._arm->toWorldCoord(robot._arm->FK(pt.positions)));  // not used
        pose_traj.push_back(pose);
    }
}

#endif
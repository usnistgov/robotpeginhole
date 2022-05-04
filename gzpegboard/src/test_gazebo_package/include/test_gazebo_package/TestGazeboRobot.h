#ifndef TESTGAZEBOROBOT
#define TESTGAZEBOROBOT

#include <vector>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"

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

struct CTaskBoardHole
{
    std::string name;
    std::string state;
    std::string holetype;
    tf::Vector3 location;
    CTaskBoardHole()
    {
        name = "empty";
    }
    CTaskBoardHole(std::string name, std::string state, std::string holetype, tf::Vector3 location)
    {
        this->name = name;
        this->state = state;
        this->holetype = holetype;
        this->location = location;
    }
};
struct CTaskBoard : public std::vector<CTaskBoardHole>
{
    CTaskBoard() {}
    CTaskBoard(const std::vector<CTaskBoardHole> &other)
    {
        copy(other.begin(), other.end(), back_inserter(*this));
    }
    CTaskBoardHole &find(std::string name)
    {
        static CTaskBoardHole empty;
        for (size_t i = 0; i < this->size(); i++)
        {
            if (this->at(i).name == name)
                return this->at(i);
        }
        return empty;
    }

    std::string toString()
    {
        std::stringstream ss;
        for (size_t i = 0; i < this->size(); i++)
            ss << this->at(i).name << ":" << this->at(i).state << ":"
               << this->at(i).location.x() << ":"
               << this->at(i).location.y() << ":"
               << this->at(i).location.z() << "\n";
        return ss.str();
    }
};
extern CTaskBoard pegs;
extern CTaskBoard holes;
extern CrpiRobotParams params;
extern crpi_robot::CrpiGazebo robot;
extern tf::Quaternion qBend;
extern  double dLengthPeg; // in meters - scale is 0.002
extern  double zMinPegArray;         // world coord in meters as reported by gazebo
extern  double zSizePegArray; // in meters (scale is 0.002 mm->meters) twice as big
extern  double zMaxPegArray;        // zMinPegArray+zSizePegArray;  // in meters using calculator
extern  double dDwellTime;  // wait between moves or control actions (for settling)
extern double xfudge;
// inline tf::Vector3 lerp(tf::Vector3 &v1, tf::Vector3 &v2, const tfScalar &t)
// {
//     return tf::Vector3(v1.m_floats[0] + (v2.m_floats[0] - v1.m_floats[0]) * t,
//                        v1.m_floats[1] + (v2.m_floats[1] - v1.m_floats[1]) * t,
//                        v1.m_floats[2] + (v2.m_floats[2] - v1.m_floats[2]) * t);
// }

inline geometry_msgs::Point geostore(double a,double b, double c)
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
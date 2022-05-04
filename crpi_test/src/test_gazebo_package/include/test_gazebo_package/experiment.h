#ifndef EXPERIMENTS_H
#define EXPERIMENTS_H

#include "test_gazebo_package/TestGazeboRobot.h"
#include <tf/tf.h>
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "test_gazebo_package/wm.h"
extern void testConversions(); // Test conversions
extern void testcurrentRobotJoints();
extern void testcurrentPose();

extern bool bBreak;
extern bool bLogFile;
extern int nLogFTVals;
extern bool bTwist;
extern bool bWiggle;
extern bool bDeadReckoning;

//extern void acquirePeg(std::string pegname);
//extern void insertPegDeadReckoning(std::string holename);
//extern void approachPegHole(std::string holename);
//extern void guardedMovePegArray(std::string holename, double ft_threahold, tf::Vector3 fudge, size_t updaterate = 10);

extern std::ostream *logfp;

inline double toDegree(double ang)
{
    return ang * 180.0 / M_PI;
}
inline double fromDegree(double ang)
{
    return ang / 180.0 * M_PI;
}
typedef enum { ROBOT_COORD=1, WORLD_COORD=2 } COORD_TYPE;
typedef enum { APROACH=1, TOP=2, BOTTOM=3 } HOLE_LOCATION;

class Experiment
{
public:
    Experiment(std::shared_ptr<crpi_robot::CRobotImpl> robot) : _robot(robot), activepeg(emptyhole)
    {
        n = 1;
        trial=0;
    }
    int n;
    int trial;
    std::shared_ptr<crpi_robot::CRobotImpl> _robot;

    void acquirePeg(std::string pegname);
    tf::Pose insertPegDeadReckoning(std::string holename);
    tf::Pose insertPegTwist (std::string holename);
    tf::Pose insertPegDeadReckoningLocation(tf::Vector3 holevector);
    void moveHoleLocation(tf::Vector3 holelocation, tf::Quaternion qbend);
    void guardedMovePegArray(std::string holename,
                             double ft_threahold,
                             tf::Vector3 fudge,
                             size_t updaterate);
    void approachPegHole(std::string holename);

    tf::Pose getHoleLocation(std::string name,
                             tf::Quaternion q,
                             tf::Vector3 dGraspedLength,
                             COORD_TYPE coord,
                             HOLE_LOCATION location);

    CPegArrayHole &activepeg; // = pegs.find(pegname);
    tf::Vector3 wldBottomPegHolePos;
    tf::Vector3 wldTopHolePos;
    tf::Vector3 wldTrueTopHolePos;
    tf::Pose wrldPegtopPose;
    tf::Pose rbtPegtopPose;
    tf::Pose rbtPegapproachPose;
    tf::Pose rbtPegretractPose;
};

extern void resetLogStream();
#endif
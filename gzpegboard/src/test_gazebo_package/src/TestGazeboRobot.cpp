// TestGazeboRobot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "TestGazeboRobot.h"
#include <test_gazebo_package/experiment.h>
#include <iostream>
//#include "conversions.h"
// #include "traj_math.h"
// #include "crpi_util.h"
// #include "crpi_gazebo.h"
// #include "tiny_kdl.h"

#pragma comment(lib, "Ws2_32.lib")
#include <random>
#include <gazebo_plugins2/GuardedMove.h>
using namespace crpi_robot;

using namespace conversion;

// Global debugging break flag
// BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
// DONT BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=0
bool bBreak = 1;

// Globals to simplify software development
CrpiRobotParams params;
crpi_robot::CrpiGazebo robot(params);
bool bDeadReckoning = false;

double dLengthPeg = 50.8 * 0.002; // in meters - scale is 0.002
// Fanuc taskboard peg array of holes
// 6.35mm=1/2 in   50.8=2in
double zMinPegArray = 0.991;         // world coord in meters as reported by gazebo
double zSizePegArray = 6.35 * 0.002; // in meters (scale is 0.002 mm->meters) twice as big
double zMaxPegArray = 1.0799;        // zMinPegArray+zSizePegArray;  // in meters using calculator
double dDwellTime = 0.0;
double xfudge = 0.02;

//#qbend: [1.0,0.0,0.0,0.0]  ikfast kinematics (world coordinates)
// qbend: [0,0.707107,0,0.707107] kdl kinematics (robot coordinates)
tf::Quaternion qBend(0, 0.707107, 0, 0.707107);

std::vector<CTaskBoardHole> p1 = {
    CTaskBoardHole("Peg1", "Full", "round", tf::Vector3(0.2697, -1.1046, 0.9307)),
    CTaskBoardHole("Peg2", "Full", "square", tf::Vector3(0.2697, -1.0538, 0.9307)),
    CTaskBoardHole("Peg3", "Full", "round", tf::Vector3(0.2697, -1.003, 0.9307)),
    CTaskBoardHole("Peg4", "Full", "square", tf::Vector3(0.2189, -1.1046, 0.941723)),
    CTaskBoardHole("Peg5", "Full", "round", tf::Vector3(0.2189, -1.0538, 0.941723)),
    CTaskBoardHole("Peg6", "Full", "square", tf::Vector3(0.21897, -1.003, 0.941723)),
    CTaskBoardHole("Peg7", "Full", "round", tf::Vector3(0.1681, -1.1046, 0.9544)),
    CTaskBoardHole("Peg8", "Full", "square", tf::Vector3(0.1681, -1.0538, 0.9544)),
    CTaskBoardHole("Peg9", "Full", "round", tf::Vector3(0.1681, -1.003, 0.9544))};
CTaskBoard pegs(p1);

std::vector<CTaskBoardHole> p2 = {
    CTaskBoardHole("Hole1", "Open", "round", tf::Vector3(0.2697, -1.3546, 0.9307)),
    CTaskBoardHole("Hole2", "Open", "square", tf::Vector3(0.2697, -1.3038, 0.9307)),
    CTaskBoardHole("Hole3", "Open", "round", tf::Vector3(0.2697, -1.253, 0.9307)),
    CTaskBoardHole("Hole4", "Open", "square", tf::Vector3(0.2189, -1.3546, 0.941723)),
    CTaskBoardHole("Hole5", "Open", "round", tf::Vector3(0.2189, -1.3038, 0.941723)),
    CTaskBoardHole("Hole6", "Open", "square", tf::Vector3(0.21897, -1.253, 0.941723)),
    CTaskBoardHole("Hole7", "Open", "round", tf::Vector3(0.1681, -1.3546, 0.9544)),
    CTaskBoardHole("Hole8", "Open", "square", tf::Vector3(0.1681, -1.3038, 0.9544)),
    CTaskBoardHole("Hole9", "Open", "round", tf::Vector3(0.1681, -1.253, 0.9544))};
CTaskBoard holes(p2);

int main(int argc, char **argv)
{
    std::cout << "Hello World!\n";
    std::cout << "******** break = " << bBreak << "\n";

    // To break: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
    std::vector<std::string> args;
    for (size_t i = 0; i < argc; ++i)
        args.push_back(argv[i]);

    bBreak = (bool)atoi(getCmdOption(args, "rosbreak:=", "1").c_str());
    bDeadReckoning = (bool)atoi(getCmdOption(args, "deadReckoning:=", "0").c_str());

    // if no break then delay for Gazebo visual to finish loading.
    while (bBreak)
    {
        // sleep 1 second
        Sleep(1000);
    }

    try
    {

        robot.init();
        //robot._arm->bDebug = true;  // turn off for now confusing

        // Hard coded for now
        robot._arm->Gripper = tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                       tf::Vector3(0.0, 0.0, .0182));

        robot._arm->GripperInv = robot._arm->Gripper.inverse();

        //-x -0.169  -y -1.140 -z 0.934191
        robot._arm->basePose = tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                        tf::Vector3(-0.169, -1.140, 0.934191));
        robot._arm->basePoseInverse = robot._arm->basePose.inverse();

#if 0
        for (size_t i = 0; i < robot._arm->jointNames.size(); i++)
            std::cout << robot._arm->jointNames[i] << ",";
        std::cout << "\n";

        // Test KDL FK/IK for fanuc 200 id
        KDL::CTinyKdlSolver::unitTest(robot._arm->robot_urdf);
#endif

#if 0
        // Test: See if we are can move gazebo robot by joint values - VISUALLY LOOK
        trajectory_msgs::JointTrajectoryPoint pt;

        for (double i = 0; i < 1000; i = i + 0.01)
        {
            std::vector<double> jts = {std::sinf(i), 0.0, 0.0, 0.0, 0.0, 0.0};
            pt.positions = jts;
            robot._arm->gzMovePt(pt);
            ros::Duration(0.1).sleep();
        }

        // Test: See if we are can move gazebo robot to a peg top and grasp
        std::cout << "Pegs\n"<< pegs.toString();

#endif

        // HOME Reset robot to home
        robot._arm->home();
        //robot.SetTool(1.0); // 1 open 0 closed

        // Test CRPI MoveToAxisTarget
        robotAxes robaxes(robot._arm->jointNames.size());
        robaxes.axis = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        robot.MoveToAxisTarget(robaxes, true);
        robot._arm->Dwell(dDwellTime);

        std::cout << "Bend = " << dumpPoseRPY(tf::Pose(qBend, tf::Vector3(0, 0, 0))) << "\n";

        // Start peg in hole assembly

        acquirePeg("Peg1");

        /////////////////////////////////////////////////
        // ********* Now try to move to empty hole
        //CTaskBoardHole &hole = holes.find("Hole1");
        if (bDeadReckoning)
        {
            insertPegDeadReckoning("Hole1");
            // tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
            // tf::Vector3 wvTopOfPegInHolePos = tf::Vector3(wBottomHolePos.x(), wBottomHolePos.y(), 1.0799);
            // tf::Pose wHole;
            // tf::Pose wHole1(qBend, wvTopOfPegInHolePos);

            // std::cout << "\n\n\nInsert Peg into empty hole\nEmpty hole1 at " << conversion::dumpVectorSimple(wBottomHolePos) << "\n";
            // // change to top of hole - why?
            // // should be same Z as peg1 - assumes bottom+peg height?

            // std::cout << "Hole1 Pose = " << conversion::dumpPoseSimple(wHole1) << "\n";

            // // Transform from world to robot space
            // tf::Pose rHole1 = robot._arm->toRobotCoord(wHole1);
            // std::cout << "Hole Robot Coord Pose = " << conversion::dumpPoseSimple(rHole1) << "\n";

            // // MOve to retract position above peg
            // tf::Pose rHoleApproach = robot._arm->retract(rHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .040));
            // std::cout << "Hole Robot Coord Retract Pose" << dumpPoseSimple(rHoleApproach) << std::endl;
            // //robot.SetAbsoluteSpeed(.01);
            // robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleApproach), true);

            // // Move to top of hole and then ungrasp (settool 1.)
            // robot.MoveTo(Convert<tf::Pose, robotPose>(rHole1), true);
            // robot.SetTool(1.0);
            // tf::Pose holeretract = robot._arm->retract(rHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060)); // kludge
            // robot.MoveTo(Convert<tf::Pose, robotPose>(holeretract), true);
            // // 1 open 0 closed
        }
        else
        {
            approachPegHole("Hole1");
            // // slowly move (guarded move) with grasped peg and read f/t sensor
            // // rostopic echo ft_sensor_topic

            // double xfudge = 0.02;
            // tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
            // // change to top of hole + fudge in z and x
            // tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + xfudge, wBottomHolePos.y(), zMaxPegArray);
            // tf::Pose wTophole1(qBend, wvTopHolePos);
            // std::cout << "TopHole1 World Coord Pose = " << conversion::dumpPoseSimple(wTophole1) << "\n";

            // // Transform from world to robot space
            // tf::Pose rTophole1 = robot._arm->toRobotCoord(wTophole1);
            // std::cout << "TopHole1 Robot Coord Pose = " << conversion::dumpPoseSimple(rTophole1) << "\n";

            // // move above oft-center hole location - have to add in length of peg
            // // xfudge already added in
            // // location in robot coordinate system
            // tf::Pose rHoleapproach = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060));
            // std::cout << "TopHole1 Robot Coord Approch = " << conversion::dumpPoseSimple(rHoleapproach) << "\n";
            // robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleapproach), true);
            // robot._arm->Dwell(dDwellTime);

            // // Now move to "touching" top of pegarray - move away in X so contact is made and obvious
            // tf::Pose rHoletouch = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg));
            // std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";
            // robot.MoveTo(Convert<tf::Pose, robotPose>(rHoletouch), true);
            // robot._arm->Dwell(dDwellTime);
#if 0
            // do a guarded move down to discernable +z force  includes x kludge
            tf::Pose rTabletop = robot._arm->toRobotCoord(tf::Pose(qBend, wvTopHolePos));
            rTabletop = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, -0.2));
            std::cout << "rTabletop Robot Coord  = " << conversion::dumpPoseSimple(rTabletop) << "\n";

            gazebo_plugins2::GuardedMove gm;
            gm.request.direction = geostore(0.0, 0.0, -1.0);
            gm.request.maxforce = geostore(0.0, 0.0, 0.05);
            ;
            std::vector<geometry_msgs::Pose> pose_traj;
            std::vector<trajectory_msgs::JointTrajectoryPoint> jt_traj;
            robot._arm->bDebug = true;
            moveit_msgs::RobotTrajectory poses = robot._arm->calcMoveTo(rTabletop, .1);
            robot._arm->bDebug = false;
            trajconvert(poses, pose_traj, jt_traj);
            gm.request.jt_traj = jt_traj;
            gm.request.pose_traj = pose_traj;
            gm.request.joint_names = robot._arm->jointNames;

            robot._arm->_gzRosGuardedMoveSrvClient.call(gm);
#else
            guardedMovePegArray("Hole1", 0.05, tf::Vector3(0.02, 0.0, 0.0), 10);
            // CTaskBoardHole &hole = holes.find("Hole1");
            // tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
            // // change to top of hole + fudge in z and x
            // tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + xfudge, wBottomHolePos.y(), zMaxPegArray);
            // tf::Pose wTophole1(qBend, wvTopHolePos);
            // tf::Pose rTophole1 = robot._arm->toRobotCoord(wTophole1);
            // tf::Pose rHoleapproach = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060));
            // tf::Pose rHoletouch = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg));

            // std::cout << "TopHole1 World Coord Pose = " << conversion::dumpPoseSimple(wTophole1) << "\n";
            // std::cout << "TopHole1 Robot Coord Pose = " << conversion::dumpPoseSimple(rTophole1) << "\n";
            // std::cout << "TopHole1 Robot Coord Approch = " << conversion::dumpPoseSimple(rHoleapproach) << "\n";
            // std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";

            // double zdist = 0.002;
            // while (ros::ok() && robot._arm->ft.force.z < 0.05)
            // {
            //     // Approach contact with table.
            //     // decrement approach to surface
            //     rHoletouch = robot._arm->retract(rHoletouch, tf::Vector3(0.0, 0.0, -0.01));
            //     std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";
            //     robot.MoveTo(Convert<tf::Pose, robotPose>(rHoletouch), true);

            //     ros::Rate(10).sleep();
            //     ros::spinOnce();
            // }
#if 0
            // We are now in contact with the table 
            // move to hole centroid where force in Z should be zeroish again.
            tf::Pose rStartPose = rHoletouch;
            tf::Pose wHole1(qBend, hole.location);
            tf::Pose rHoleTop = robot._arm->toRobotCoord(wHole1);
            rHoleTop.getOrigin().setZ(rStartPose.getOrigin().z());
            double n=0.0;
            while (ros::ok() && fabs(robot._arm->ft.force.z) > 0.001)
            {
                std::cout << "Force =" << robot._arm->ft.force.x << ","
                << robot._arm->ft.force.y << "," 
                << robot._arm->ft.force.z << std::endl;

                tf::Vector3 diff;
                n=n+0.1;
                diff=lerp(rStartPose.getOrigin(),rHoleTop.getOrigin(), n);
                std::cout << "Lerp  End=" << conversion::dumpVectorSimple(rHoleTop.getOrigin()) 
                << " start=" << conversion::dumpVectorSimple(rStartPose.getOrigin()) 
                << " diff=" <<  conversion::dumpVectorSimple(diff) << "\n";
                tf::Pose rNextPose (qBend, diff);
                std::cout << "Guarded Move Robot Coord = " << conversion::dumpPoseSimple(rNextPose) << "\n";
                robot.MoveTo(Convert<tf::Pose, robotPose>(rNextPose), true);

                ros::Rate(10).sleep();
                ros::spinOnce();
            }
#endif
            // Now we should move in -z until we detect a discernable positive z force.
            std::cout << "Force Z=" << robot._arm->ft.force.z << std::endl;

#endif
        }
    }
    catch (...)
    {
        std::cerr << "Main program exception \n";
    }
    std::cout << "Goodbye World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

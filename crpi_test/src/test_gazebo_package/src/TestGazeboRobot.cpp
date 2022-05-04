// TestGazeboRobot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// catkin_make --only-pkg-with-deps robocalc
// catkin_make --only-pkg-with-deps crpi_math
// catkin_make --only-pkg-with-deps orocos_kdl
// catkin_make --only-pkg-with-deps crpi
// catkin_make --only-pkg-with-deps ulapi
// catkin_make --only-pkg-with-deps motion_prims
// catkin_make --only-pkg-with-deps guardedmove_plugin
// catkin_make -DCATKIN_WHITELIST_PACKAGES=""
// Methods to use rosrun to invoke experiments
// Global debugging break flag
// BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
// DONT BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=0

// Use Visual Code to attach debugger to looping break point
// rosrun test_gazebo_package test_gazebo_package rosbreak:=1

// Use clode w/wout debug break in visual code to log F/T experiments to
// see standard deviation. Possibly add ini/yaml file to add tests.
// rosrun test_gazebo_package test_gazebo_package rosbreak:=1  logft:=3

// No breaking for dead reckoning demonstration of ROS/Gazebo code
// rosrun test_gazebo_package test_gazebo_package rosbreak:=0 deadReckoning:=1

// Tilted approach of hole
// rosrun test_gazebo_package test_gazebo_package rosbreak:=1 test:=tilt

//rosrun test_gazebo_package test_gazebo_package rosbreak:=1 test:=rpy
//rosrun test_gazebo_package test_gazebo_package rosbreak:=1 twist:=1

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
//#include <std_srvs/Empty.h>

using namespace crpi_robot;

using namespace conversion;



std::string sTest = "none";
// Globals to simplify software development
CrpiRobotParams params;
crpi_robot::CrpiGazebo robot(params);

//#qbend: [1.0,0.0,0.0,0.0]  ikfast kinematics (world coordinates)
// qbend: [0 0.707107 0 0.707107] kdl kinematics (robot coordinates)
// > q 0 110.0 0
// [0.000000,0.819152,0.000000,0.573576]
// > q -20. 90.0 0
// [-0.122788,0.696364,0.122788,0.696364]
tf::Quaternion qBend; //(0, 0.707107, 0, 0.707107);



int main(int argc, char **argv)
{
    CPegArray holearray("insertion.pegholesarray");
    CPegArray pegarray("supply.holepegarray");

    std::cout << "Hello World!\n";
    std::cout << "******** break = " << bBreak << "\n";

    // To break: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
    std::vector<std::string> args;
    for (size_t i = 0; i < argc; ++i)
        args.push_back(argv[i]);

    // rosrun command line options
    bBreak = (bool)atoi(getCmdOption(args, "rosbreak:=", "1").c_str());
    bDeadReckoning = (bool)atoi(getCmdOption(args, "deadReckoning:=", "0").c_str());
    bLogFile = (bool)atoi(getCmdOption(args, "logfile:=", "1").c_str());
    nLogFTVals = (int)atoi(getCmdOption(args, "logft:=", "0").c_str());
    bWiggle = (int)atoi(getCmdOption(args, "wiggle:=", "0").c_str());
    bTwist = (int)atoi(getCmdOption(args, "twist:=", "0").c_str());
    sTest = getCmdOption(args, "test:=", "none");

    // if no break then delay for Gazebo visual to finish loading.
    while (bBreak)
    {
        // sleep 1 second
        Sleep(1000);
    }
#ifdef PEGARRAY
    holearray.setCentroid(tf::Vector3(10.,20.,30.));
    std::cout << holearray.toString();
    pegarray.setCentroid(tf::Vector3(30.,60.,90.));
    std::cout << pegarray.toString();
#endif  
    try
    {
        robot.init();
        //robot._arm->bDebug = true;  // turn off for now confusing

        // Hard coded for now - gripper orientation and length
        robot._arm->Gripper = tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                       tf::Vector3(0.0, 0.0, .0182));

        robot._arm->GripperInv = robot._arm->Gripper.inverse();

        robot._arm->Grasped = tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                                       tf::Vector3(0.0, 0.0, 50.8 * gzPegboardScaleFactor));

        robot._arm->GraspedInv = robot._arm->Grasped.inverse();

        // Robot offset from world coordinate space
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
#ifdef PEGARRAY
        while(!robot._wm->isReadModels() )
        {
            Sleep(100);
        }
        std::cout << robot._wm->toString();
#endif
        // assign cetnroid for offset caluclations of pegarrays pegs/holes in supply/dest pegboards
        CWm supplypegarraywm;
        CWm supplybasearraywm;
        CWm destpegarraywm;
        CWm destbasearraywm;
        try
        {
            if (!robot._wm->getInstance("sku_taskfeeder_base_fanuc1", supplybasearraywm))
                throw std::string("getInstance sku_taskfeeder_base_fanuc1 failed");

            if (!robot._wm->getInstance("sku_taskfeeder_pegarray_fanuc1", supplypegarraywm))
                throw std::string("getInstance sku_taskfeeder_pegarray_fanuc1 failed");

            if (!robot._wm->getInstance("sku_taskfeeder_base_fanuc2", destbasearraywm))
                throw std::string("getInstance sku_taskfeeder_base_fanuc2 failed");

            if (!robot._wm->getInstance("sku_taskfeeder_pegarray_fanuc2", destpegarraywm))
                throw std::string("getInstance sku_taskfeeder_pegarray_fanuc2 failed");

            supplybasearray.setCentroid(supplybasearraywm._location);
            supplypegarray.setCentroid(supplypegarraywm._location);

            destbasearray.setCentroid(destbasearraywm._location);
            destpegarray.setCentroid(destpegarraywm._location);
        }
        catch (std::string errmsg)
        {
        }

        /////////////////////////////////////////////////
        // ********* Now try to move to empty hole
        if ((sTest.compare(0, strlen("rpy"), "rpy") == 0))
        {
            Experiment exp(robot._arm);
            tf::Quaternion bend;

            robot._arm->bDebug = true;

            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            exp.acquirePeg("Peg1");

            bend.setRPY(0., fromDegree(-20.), 0.);
            bend = qBend * bend;
            //rTestApproach=tf::Pose(bend, rApproachHole.getOrigin());
            //robot._arm->moveTraj(robot._arm->currentPose(), rTestApproach, 0.9);
            bend = qBend;
            tf::Pose wBottomHoleLocation = exp.getHoleLocation("Hole1", bend, tf::Vector3(0, 0, 0), WORLD_COORD, BOTTOM);
            CPegArrayHole &hole = holes.find("Hole1");
            std::cout << "RPY wBottomHoleLocation=" << conversion::dumpPoseSimple(wBottomHoleLocation) << "\n";
            std::cout << "RPY hole.location=" << conversion::dumpVectorSimple(hole.location) << "\n";
            exp.moveHoleLocation(hole.location, bend);
        }
        else if (bDeadReckoning)
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            Experiment exp(robot._arm);
            exp.acquirePeg("Peg1");
            exp.insertPegDeadReckoning("Hole1");
        }
        else if (bTwist)
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            Experiment exp(robot._arm);
            exp.acquirePeg("Peg1");
            exp.insertPegTwist("Hole1");
            // same position, different orientation

        }
        else if (bWiggle)
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            Experiment exp(robot._arm);
            exp.acquirePeg("Peg1");
            exp.insertPegDeadReckoning("Hole1");
            // mild miss, with dither into hole using force
        }
        else if (nLogFTVals > 0)
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            bLogFile = true; // set so reset log stream does new file?
            for (size_t i = 0; i < nLogFTVals; i++)
            {
                // reset up log file naming
                resetLogStream();

                Experiment exp(robot._arm);
                exp.n = 1; // reset experiment logging time serieies variable to 1
                exp.trial++;
                exp.acquirePeg("Peg1");
                exp.approachPegHole("Hole1");
                exp.guardedMovePegArray("Hole1", 0.025, tf::Vector3(0.02, 0.0, 0.0), 10);

                exp.approachPegHole("Hole1"); // retract from hole
                CPegArrayHole &peghole = pegs.find("Peg1");
                exp.insertPegDeadReckoningLocation(peghole.location);
                robot._arm->home();
            }
        }
        else if ((sTest.compare(0, strlen("tilt"), "tilt") == 0))
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            Experiment exp(robot._arm);
            exp.acquirePeg("Peg1");

            // > q 0 110.0 0
            // [0.000000,0.819152,0.000000,0.573576]
            qBend = tf::Quaternion(0.000000, 0.819152, 0.000000, 0.573576);
            exp.approachPegHole("Hole1");
            // Approach to near hole surface
            // use FT readings to stop z descent toward surface
            // Simulated fudge - all zeros is dead reckoning
            tf::Vector3 fudge(0.0, 0.0, 0.0);
            // guardedMovePegArray holename, ft_threahold,  fudge, updaterate
            exp.guardedMovePegArray("Hole1", 0.025, fudge, 10);
        }
        else
        {
            qBend = tf::Quaternion(0, 0.707107, 0, 0.707107);
            resetLogStream();

            Experiment exp(robot._arm);
            exp.acquirePeg("Peg1");
            exp.approachPegHole("Hole1");
            exp.guardedMovePegArray("Hole1", 0.025, tf::Vector3(0.02, 0.0, 0.0), 10);

            exp.approachPegHole("Hole1"); // retract from hole

            // Reset peg world to original setup
            CPegArrayHole &peghole = pegs.find("Peg1");
            exp.insertPegDeadReckoningLocation(peghole.location);
            robot._arm->home();

            // Now we should move in -z until we detect a discernable positive z force.
            std::cout << "Force Z=" << robot._arm->ft.force.z << std::endl;
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

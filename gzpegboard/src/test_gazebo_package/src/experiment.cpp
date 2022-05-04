

#include "test_gazebo_package/experiment.h"
#include <test_gazebo_package/conversions.h>
#include <tf/tf.h>
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"
//#include <motion_prims/AssemblyPrims.h>

using namespace crpi_robot;
using namespace conversion;

// CRPI  application of peg in hole using raster micromotion
#if 0
void raster()
{
      //   temp = ap.totalLength / ap.lengthDelta;
      localCount = counter % (int)(2.0f * temp);

      if (localCount > temp)
      {
        localCount = (int)(2.0f * temp) - localCount;
      }

      position = localCount * ap.lengthDelta;
      ratio = position / (ap.width + ap.lengthStep);
      
      temp = decimal(ratio);
      wholeval = whole(ratio);

      if (temp > ap.rasterRatio)
      {
        //! In the raster step
        if (even(wholeval))
        {
          deltas.y = ap.width;
        }
        else
        {
          deltas.y = 0.0f;
        }
        deltas.x = (wholeval + ((temp - ap.rasterRatio) / (1.0 - ap.rasterRatio))) * ap.lengthStep;
      }
      else
      {
        //! On the raster
        if (even(wholeval))
        {
          deltas.y = (temp / ap.rasterRatio) * ap.width;
        }
        else
        {
          deltas.y = (1.0f - (temp / ap.rasterRatio)) * ap.width;
        }
        deltas.x = wholeval * ap.lengthStep;
      }
      break;

}
#endif
void testConversions()
{
    tf::Pose testpose = tf::Pose(tf::Quaternion(0, 0.707107, 0, 0.707107), tf::Vector3(0.2697, -1.3546, 0.9307));
    std::cout << "Test Pose = " << conversion::dumpPoseSimple(testpose) << "\n";
    robotPose robotpose = Convert<tf::Pose, robotPose>(testpose);
    std::cout << "Test robotPose = " << robotpose.print() << "\n";
    testpose = conversion::Convert<robotPose, tf::Pose>(robotpose);
    std::cout << "Converted robotPose2tf = " << conversion::dumpPoseSimple(testpose) << "\n";
}
void testcurrentRobotJoints()
{
    // Test: See if we are getting robot joints
    std::vector<double> jnts = robot._arm->currentRobotJoints();
    std::cout << "Current Joints=" << vectorDump<double>(jnts) << "\n";
}
void testcurrentPose()
{
    // Test: See if we are getting robot pose
    tf::Pose p = robot._arm->currentPose();
    std::cout << "Current Pose = " << conversion::dumpPoseSimple(p) << "\n";
    std::cout << "Current Pose = " << conversion::dumpPoseRPY(p) << "\n";
}
void acquirePeg(std::string pegname)
{
    CTaskBoardHole &peg = pegs.find("Peg1");
    tf::Vector3 bottomHolePos = peg.location; // bottom of taskboard shelf
    std::cout << "Peg1 Bottom World Coord " << conversion::dumpVectorSimple(bottomHolePos) << "\n";

    // change to top of hole actually a fudge value?
    tf::Vector3 topHolePos = tf::Vector3(bottomHolePos.x(), bottomHolePos.y(), 1.0799);
    std::cout << "Peg1 Top World coord  = " << conversion::dumpVectorSimple(topHolePos) << "\n";
    tf::Vector3 trueTopHolePos = tf::Vector3(bottomHolePos.x(), bottomHolePos.y(), bottomHolePos.z() + dLengthPeg);
    std::cout << "Peg1 actual Top in World Coord = " << conversion::dumpVectorSimple(trueTopHolePos) << "\n";

    // grasp top of the peg
    tf::Pose pegtop(qBend, topHolePos);

    // Transform rfrom world to robot space
    pegtop = robot._arm->toRobotCoord(pegtop);
    std::cout << "Peg1 Robot Coord Pose = " << conversion::dumpPoseSimple(pegtop) << "\n";

    tf::Pose pegapproach = robot._arm->retract(pegtop, tf::Vector3(0.0, 0.0, .020));
    std::cout << "Peg1 Robot Coord Retract Pose" << dumpPoseSimple(pegapproach) << std::endl;
    robot.MoveTo(Convert<tf::Pose, robotPose>(pegapproach), true);
    robot._arm->Dwell(dDwellTime);

    //robot.SetAbsoluteSpeed(.1);
    robot.MoveTo(Convert<tf::Pose, robotPose>(pegtop), true);
    robot._arm->Dwell(dDwellTime);
    robot.SetTool(0.0); // 1 open 0 closed
    robot._arm->Dwell(dDwellTime);

    //robot.SetAbsoluteSpeed(.1);
    tf::Pose pegretract = robot._arm->retract(pegtop, tf::Vector3(0.0, 0.0, dLengthPeg + .040));
    robot.MoveTo(Convert<tf::Pose, robotPose>(pegretract), true);
    robot._arm->Dwell(dDwellTime);
}

void insertPegDeadReckoning(std::string holename)
{
    CTaskBoardHole &hole = holes.find(holename);
    tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
    tf::Vector3 wvTopOfPegInHolePos = tf::Vector3(wBottomHolePos.x(), wBottomHolePos.y(), 1.0799);
    tf::Pose wHole;
    tf::Pose wHole1(qBend, wvTopOfPegInHolePos);

    std::cout << "\n\n\nInsert Peg into empty hole\nEmpty hole1 at " << conversion::dumpVectorSimple(wBottomHolePos) << "\n";
    // change to top of hole - why?
    // should be same Z as peg1 - assumes bottom+peg height?

    std::cout << "Hole1 Pose = " << conversion::dumpPoseSimple(wHole1) << "\n";

    // Transform from world to robot space
    tf::Pose rHole1 = robot._arm->toRobotCoord(wHole1);
    std::cout << "Hole Robot Coord Pose = " << conversion::dumpPoseSimple(rHole1) << "\n";

    // MOve to retract position above peg
    tf::Pose rHoleApproach = robot._arm->retract(rHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .040));
    std::cout << "Hole Robot Coord Retract Pose" << dumpPoseSimple(rHoleApproach) << std::endl;
    //robot.SetAbsoluteSpeed(.01);
    robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleApproach), true);

    // Move to top of hole and then ungrasp (settool 1.)
    robot.MoveTo(Convert<tf::Pose, robotPose>(rHole1), true);
    robot.SetTool(1.0);
    tf::Pose holeretract = robot._arm->retract(rHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060)); // kludge
    robot.MoveTo(Convert<tf::Pose, robotPose>(holeretract), true);
    // 1 open 0 closed
}
void approachPegHole(std::string holename)
{
    CTaskBoardHole &hole = holes.find(holename);
    double xfudge = 0.02;
    tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
    // change to top of hole + fudge in z and x
    tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + xfudge, wBottomHolePos.y(), zMaxPegArray);
    tf::Pose wTophole1(qBend, wvTopHolePos);
    std::cout << "TopHole1 World Coord Pose = " << conversion::dumpPoseSimple(wTophole1) << "\n";

    // Transform from world to robot space
    tf::Pose rTophole1 = robot._arm->toRobotCoord(wTophole1);
    std::cout << "TopHole1 Robot Coord Pose = " << conversion::dumpPoseSimple(rTophole1) << "\n";

    // move above oft-center hole location - have to add in length of peg
    // xfudge already added in
    // location in robot coordinate system
    tf::Pose rHoleapproach = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060));
    std::cout << "TopHole1 Robot Coord Approch = " << conversion::dumpPoseSimple(rHoleapproach) << "\n";
    robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleapproach), true);
    robot._arm->Dwell(dDwellTime);

    // Now move to "touching" top of pegarray - move away in X so contact is made and obvious
    tf::Pose rHoletouch = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg));
    std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";
    robot.MoveTo(Convert<tf::Pose, robotPose>(rHoletouch), true);
    robot._arm->Dwell(dDwellTime);
}
void guardedMovePegArray(std::string holename, double ft_threahold, tf::Vector3 fudge, size_t updaterate)
{
    CTaskBoardHole &hole = holes.find(holename);
    tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
    // change to top of hole + fudge in z and x
    tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + fudge.x(), wBottomHolePos.y(), zMaxPegArray);
    tf::Pose wTophole1(qBend, wvTopHolePos);
    tf::Pose rTophole1 = robot._arm->toRobotCoord(wTophole1);
    tf::Pose rHoleapproach = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060));
    tf::Pose rHoletouch = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg));
    tf::Pose rGuardFinal = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, -dLengthPeg));

    std::cout << "TopHole1 World Coord Pose = " << conversion::dumpPoseSimple(wTophole1) << "\n";
    std::cout << "TopHole1 Robot Coord Pose = " << conversion::dumpPoseSimple(rTophole1) << "\n";
    std::cout << "TopHole1 Robot Coord Approch = " << conversion::dumpPoseSimple(rHoleapproach) << "\n";
    std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";

    double zdist = -0.002;
    size_t iteration = 0;
    moveit_msgs::RobotTrajectory traj = robot._arm->calcTraj(robot._arm->currentPose(), rGuardFinal, 0.01);
    while (ros::ok() && robot._arm->ft.force.z < ft_threahold) // 0.05)
    {
        if (iteration >= traj.joint_trajectory.points.size())
            break;

        trajectory_msgs::JointTrajectoryPoint pt = traj.joint_trajectory.points[iteration];
        robot._arm->gzMovePt(pt);
        // Approach contact with table.
        // decrement approach to surface
        //rHoletouch = robot._arm->retract(rHoletouch, tf::Vector3(0.0, 0.0, -0.002)); // -0.01));
        //std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";
        // robot.MoveTo(Convert<tf::Pose, robotPose>(rHoletouch), true);

        //ros::Rate(updaterate).sleep();
        std::this_thread::sleep_for(std::chrono::milliseconds(updaterate));

        ros::spinOnce();
        ++iteration;
        std::cout << "Force Z=" << robot._arm->ft.force.z << std::endl;
    }
}

void guardedMovePegHole(std::string holename, size_t updaterate)
{
    CTaskBoardHole &hole = holes.find(holename);
    tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
    // change to top of hole + fudge in z and x
    tf::Vector3 wvActualTopHolePos = tf::Vector3(wBottomHolePos.x() , wBottomHolePos.y(), zMaxPegArray);
    tf::Pose wpActualTophole1(qBend, wvActualTopHolePos);
    tf::Pose rpActualTophole1 = robot._arm->toRobotCoord(wpActualTophole1);
    // Assembly assembly;

    // if( assembly.AddSearchRaster (10, 20.0 /*mm*/,  60.0 /*mm*/, 100.0 /*speed (mm/s)*/)
    // !=   CanonReturn)
    // {

    // }

}
 
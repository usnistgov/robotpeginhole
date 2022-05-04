

#include "test_gazebo_package/experiment.h"
#include <test_gazebo_package/conversions.h>
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"
#include <boost/algorithm/string/predicate.hpp>

// Big compile hurdle for now
//#include <motion_prims/AssemblyPrims.h>

using namespace crpi_robot;
using namespace conversion;
std::ostream *logfp = &cout;



bool bBreak = 1;
bool bLogFile = 1;
int nLogFTVals = 5;
bool bTwist = 0;
bool bWiggle = 0;
bool bDeadReckoning = false;

void resetLogStream()
{
  // resetup logging file
  std::ofstream fout;
  std::string path = getRosLogTimeFilestamp("test_gazebo_package", "ft");
  if (bLogFile)
  {
    if (fout.is_open())
      fout.close();
    fout.open(path.c_str(), std::ofstream::out);
    if (!fout.bad())
      logfp = &fout;
  }
}
// void raster()
// {
//       //   temp = ap.totalLength / ap.lengthDelta;
//       localCount = counter % (int)(2.0f * temp);

//       if (localCount > temp)
//       {
//         localCount = (int)(2.0f * temp) - localCount;
//       }

//       position = localCount * ap.lengthDelta;
//       ratio = position / (ap.width + ap.lengthStep);

//       temp = decimal(ratio);
//       wholeval = whole(ratio);

//       if (temp > ap.rasterRatio)
//       {
//         //! In the raster step
//         if (even(wholeval))
//         {
//           deltas.y = ap.width;
//         }
//         else
//         {
//           deltas.y = 0.0f;
//         }
//         deltas.x = (wholeval + ((temp - ap.rasterRatio) / (1.0 - ap.rasterRatio))) * ap.lengthStep;
//       }
//       else
//       {
//         //! On the raster
//         if (even(wholeval))
//         {
//           deltas.y = (temp / ap.rasterRatio) * ap.width;
//         }
//         else
//         {
//           deltas.y = (1.0f - (temp / ap.rasterRatio)) * ap.width;
//         }
//         deltas.x = wholeval * ap.lengthStep;
//       }
//       break;

// }
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

void Experiment::acquirePeg(std::string pegname)
{
  //CTaskBoardHole &peg = pegs.find(pegname);

  // Peg1 World defined Bottom World Coord    0.27,  -1.10,   0.93,
  // Peg1 Offset defined Bottom World Coord    0.27,  -1.10,   1.11,
  activepeg = pegs.find(pegname);
  wldBottomPegHolePos = activepeg.location; // bottom of taskboard shelf
  std::cout << pegname << " World defined Bottom World Coord " << conversion::dumpVectorSimple(wldBottomPegHolePos) << "\n";

  CPegArrayHole activeoffsetpeg = supplybasearray.find("Offset"+pegname);
  tf::Vector3 offBottonPegHolePos = activeoffsetpeg.curTopZLocation(); // uses centroid and offset based location
  std::cout << pegname << " Offset defined Bottom World Coord " << conversion::dumpVectorSimple(offBottonPegHolePos) << "\n";

  // change to top of hole KLUDGE: actually a fudge value?
  wldTopHolePos = tf::Vector3(wldBottomPegHolePos.x(), wldBottomPegHolePos.y(), 1.0799); // 1.0799 where is this from?
  std::cout << pegname << " Top Hole World coord  = " << conversion::dumpVectorSimple(wldTopHolePos) << "\n";

  // activepeg = supplypegarray.find(pegname);
  tf::Vector3 pegarrayboard;
  CPegArrayHole activeoffsethole = supplypegarray.find("Offset"+pegname);
  tf::Vector3 offTopPegHolePos = activeoffsethole.curTopZLocation(); // uses centroid and offset based location
  std::cout << pegname << " Offset Top Hole World coord  = " << conversion::dumpVectorSimple(offTopPegHolePos) << "\n";

  // This should be top of peg but doesn't seem to work. :(
  // off by 0.05 meters?
  wldTrueTopHolePos = tf::Vector3(wldBottomPegHolePos.x(), wldBottomPegHolePos.y(), wldBottomPegHolePos.z() + dLengthPeg);
  std::cout << pegname << " actual Top in World Coord = " << conversion::dumpVectorSimple(wldTrueTopHolePos) << "\n";
  if (trial > 1)
    wldTopHolePos = wldTrueTopHolePos;
  // grasp top of the peg
  wrldPegtopPose = tf::Pose(qBend, wldTopHolePos);

  // Transform rfrom world to robot space
  rbtPegtopPose = robot._arm->toRobotCoord(wrldPegtopPose);
  std::cout << pegname << " Robot Coord Pose = " << conversion::dumpPoseSimple(rbtPegtopPose) << "\n";

  rbtPegapproachPose = robot._arm->retract(rbtPegtopPose, tf::Vector3(0.0, 0.0, .020));
  std::cout << pegname << " Robot Coord Retract Pose" << dumpPoseSimple(rbtPegapproachPose) << std::endl;
  robot.MoveTo(Convert<tf::Pose, robotPose>(rbtPegapproachPose), true);
  robot._arm->Dwell(dDwellTime);

  //robot.SetAbsoluteSpeed(.1);
  robot.MoveTo(Convert<tf::Pose, robotPose>(rbtPegtopPose), true);
  robot._arm->Dwell(dDwellTime);
  robot.SetTool(0.0); // 1 open 0 closed
  robot._arm->Dwell(dDwellTime);

  //robot.SetAbsoluteSpeed(.1);
  rbtPegretractPose = robot._arm->retract(rbtPegtopPose, tf::Vector3(0.0, 0.0, dLengthPeg + .040));
  robot.MoveTo(Convert<tf::Pose, robotPose>(rbtPegretractPose), true);
  robot._arm->Dwell(dDwellTime);

  // update current position of the peg - FIXME: should be world?
  // FIXME: we assume all is honky dory getting here....
  activepeg.wCurlocation = robot._arm->toWorldCoord(rbtPegretractPose).getOrigin();
}

tf::Pose Experiment::insertPegDeadReckoning(std::string holename)
{
  CPegArrayHole &hole = holes.find(holename);
  tf::Pose rbtHole1 = insertPegDeadReckoningLocation(hole.location);
  robot.SetTool(1.0);
  tf::Pose holeretract = robot._arm->retract(rbtHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060)); // kludge
  robot.MoveTo(Convert<tf::Pose, robotPose>(holeretract), true);
  return rbtHole1;
}
tf::Pose Experiment::insertPegTwist(std::string holename)
{
  CPegArrayHole &hole = holes.find(holename);
  tf::Pose rbtHole1 = insertPegDeadReckoningLocation(hole.location);
  tf::Pose holetwist;
  tf::Quaternion bend;
  int n = 0;
  std::stringstream ss;
  ss << n++ << dumpPoseRPY(rbtHole1);
  ss << "," << robot._arm->ft.force.x << "," << robot._arm->ft.force.y << "," << robot._arm->ft.force.z << "\n";
  //    ROS_INFO_STREAM_NAMED("twistdata", ss.str());
  *logfp << ss.str() << std::endl
         << std::flush;
  Sleep(4000);
  for (double t = 1.; t < 10.; t = t + 1.)
  {
    //  resetLogStream();
    bend.setRPY(0., fromDegree(-1. * t), 0.);
    bend = qBend * bend;

    tf::Pose holetwist = tf::Pose(bend, rbtHole1.getOrigin());
    robot.MoveTo(Convert<tf::Pose, robotPose>(holetwist), true);
    std::stringstream ss;
    ss << n++ << dumpPoseRPY(holetwist);
    ss << "," << robot._arm->ft.force.x << "," << robot._arm->ft.force.y << "," << robot._arm->ft.force.z << "\n";
    //    ROS_INFO_STREAM_NAMED("twistdata", ss.str());
    *logfp << ss.str() << std::endl
           << std::flush;
  }
  return holetwist;
}
void Experiment::approachPegHole(std::string holename)
{
  CPegArrayHole &hole = holes.find(holename);
  double xfudge = 0.02;
  tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
  // change to top of hole + fudge in z and x
  tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + xfudge, wBottomHolePos.y(), zWrldMaxPegArray);
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

void Experiment::guardedMovePegArray(std::string holename, double ft_threahold, tf::Vector3 fudge, size_t updaterate)
{
  CPegArrayHole &hole = holes.find(holename);

  // bottom of taskboard shelf from peg insertion
  tf::Vector3 wBottomHolePos = hole.location;

  // change to top of hole + fudge in z and x
  tf::Vector3 wvTopHolePos = tf::Vector3(wBottomHolePos.x() + fudge.x(), wBottomHolePos.y(), zWrldMaxPegArray);
  tf::Pose wTophole1(qBend, wvTopHolePos);
  tf::Pose rTophole1 = robot._arm->toRobotCoord(wTophole1);
  tf::Pose rHoleapproach = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg + .060));
  tf::Pose rHoletouch = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg));
  // this hits amost the bottom of the hole - the tier
  tf::Pose rGuardFinal = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, -dLengthPeg));
  // subtract off grasped end which raises current peg lowest Z
  tf::Pose rGuardApproachFinal = robot._arm->retract(rTophole1, tf::Vector3(0.0, 0.0, dLengthPeg - 0.04));

  //std::cout << "TopHole1 World Coord Pose = " << conversion::dumpPoseSimple(wTophole1) << "\n";

  std::cout << "Length of Peg  = " << dLengthPeg << "\n";
  std::cout << "TopHole1 Robot Coord Pose = " << conversion::dumpPoseSimple(rTophole1) << "\n";
  std::cout << "TopHole1 Robot Coord Approch = " << conversion::dumpPoseSimple(rHoleapproach) << "\n";
  std::cout << "Holetouch Robot Coord Near = " << conversion::dumpPoseSimple(rHoletouch) << "\n";
  std::cout << "GuardedFinal Robot Coord  = " << conversion::dumpPoseSimple(rGuardFinal) << "\n";
  std::cout << "GuardedFinal Robot Coord Approach = " << conversion::dumpPoseSimple(rGuardApproachFinal) << "\n";
  std::cout << "Current Robot Pos = " << conversion::dumpPoseSimple(robot._arm->currentPose()) << "\n";

  // Approach to near hole surface
  robot._arm->moveTraj(robot._arm->currentPose(), rGuardApproachFinal, 0.9);

  // use FT readings to stop z descent toward surface
  double zdist = -0.002; // amount of z descent each cycle
  size_t iteration = 0;
  moveit_msgs::RobotTrajectory traj = robot._arm->calcTraj(rGuardApproachFinal, rGuardFinal, 0.01);
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
    tf::Pose curpose = robot._arm->FK(pt.positions);

    std::stringstream ss;
    ss << n++ << "," << curpose.getOrigin().x() << "," << curpose.getOrigin().y() << "," << curpose.getOrigin().z() << "," << robot._arm->ft.force.x << "," << robot._arm->ft.force.y << "," << robot._arm->ft.force.z;
    ROS_INFO_STREAM_NAMED("tfdata", ss.str());
    *logfp << ss.str() << std::endl
           << std::flush;
  }
}

// void guardedMovePegHole(std::string holename, size_t updaterate)
// {
//     CTaskBoardHole &hole = holes.find(holename);
//     tf::Vector3 wBottomHolePos = hole.location; // bottom of taskboard shelf from peg insertion
//     // change to top of hole + fudge in z and x
//     tf::Vector3 wvActualTopHolePos = tf::Vector3(wBottomHolePos.x() , wBottomHolePos.y(), zMaxPegArray);
//     tf::Pose wpActualTophole1(qBend, wvActualTopHolePos);
//     tf::Pose rpActualTophole1 = robot._arm->toRobotCoord(wpActualTophole1);
//     Assembly assembly;

//     if( assembly.AddSearchRaster (10, 20.0 /*mm*/,  60.0 /*mm*/, 100.0 /*speed (mm/s)*/)
//     !=   CanonReturn)
//     {

//     }

// }

tf::Pose Experiment::getHoleLocation(std::string name,
                                     tf::Quaternion q,
                                     tf::Vector3 dGraspedLength,
                                     COORD_TYPE coord,
                                     HOLE_LOCATION location)
{
  CPegHoleArray *tb = &holes;
  std::string pbname(name);
  std::transform(pbname.begin(), pbname.end(), pbname.begin(), ::tolower);
  if (boost::starts_with(pbname, ("peg")))
  {
    tb = &pegs;
  }
  CPegArrayHole &hole = tb->find(name);

  // bottom of taskboard shelf from peg insertion
  tf::Vector3 wvBottomHolePos = hole.location;
  tf::Vector3 wvTopHolePos = tf::Vector3(wvBottomHolePos.x(), wvBottomHolePos.y(), zWrldMaxPegArray);
  tf::Pose wBottomhole1(q, wvBottomHolePos);
  tf::Pose wTophole1(q, wvTopHolePos);
  tf::Pose wHoleapproach = robot._arm->retract(wTophole1, tf::Vector3(0.0, 0.0, dGraspedLength.z() + .060));

  if (coord == WORLD_COORD)
  {
    switch (location)
    {
    case APROACH:
      return wHoleapproach;
    case TOP:
      return wTophole1;
    case BOTTOM:
      return wBottomhole1;
    }
  }
  else if (coord == ROBOT_COORD)
  {
    switch (location)
    {
    case APROACH:
      return robot._arm->toRobotCoord(wHoleapproach);
    case TOP:
      return robot._arm->toRobotCoord(wTophole1);
    case BOTTOM:
      return robot._arm->toRobotCoord(wBottomhole1);
    }
  }
  return wHoleapproach;
}

tf::Pose Experiment::insertPegDeadReckoningLocation(tf::Vector3 holelocation)
{

  tf::Vector3 wBottomHolePos = holelocation; // hole.location; // bottom of taskboard shelf from peg insertion
  tf::Vector3 wvTopOfPegInHolePos = tf::Vector3(wBottomHolePos.x(), wBottomHolePos.y(), 1.0799);
  tf::Pose wrldHole1(qBend, wvTopOfPegInHolePos);

  std::cout << "\n\n\nInsert Peg into empty hole\nEmpty hole1 at " << conversion::dumpVectorSimple(wBottomHolePos) << "\n";
  // change to top of hole - why?
  // should be same Z as peg1 - assumes bottom+peg height?

  std::cout << "Hole1 Pose = " << conversion::dumpPoseSimple(wrldHole1) << "\n";

  // Transform from world to robot space
  tf::Pose rbtHole1 = robot._arm->toRobotCoord(wrldHole1);
  std::cout << "Hole Robot Coord Pose = " << conversion::dumpPoseSimple(rbtHole1) << "\n";

  // MOve to retract position above hole
  tf::Pose rHoleApproach = robot._arm->retract(rbtHole1, tf::Vector3(0.0, 0.0, dLengthPeg + .040));
  std::cout << "Hole Robot Coord Retract Pose" << dumpPoseSimple(rHoleApproach) << std::endl;
  //robot.SetAbsoluteSpeed(.01);
  robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleApproach), true);

  // Move to top of hole and then ungrasp (settool 1.)
  // use of peg length to reach bottom of hole.
  // So robot end-effector pose is approximately top of hole - AND -
  // peg legth makes up distance to bottom of hole.
  robot.MoveTo(Convert<tf::Pose, robotPose>(rbtHole1), true);
  return rbtHole1;
  // 1 open 0 closed
}

void Experiment::moveHoleLocation(tf::Vector3 holelocation, tf::Quaternion qbend)
{

  tf::Vector3 wBottomHolePos = holelocation; // hole.location; // bottom of taskboard shelf from peg insertion
  // tf::Vector3 wvTopOfPegInHolePos = tf::Vector3(wBottomHolePos.x(), wBottomHolePos.y(), 1.0799);
  tf::Vector3 wvTopOfPegInHolePos = tf::Vector3(wBottomHolePos.x(), wBottomHolePos.y(), .991);
  tf::Pose wrldHole1(qbend, wvTopOfPegInHolePos);

  std::cout << "\n\n\nMove Peg to empty hole location\nEmpty hole1 at " << conversion::dumpVectorSimple(wBottomHolePos) << "\n";
  // change to top of hole - why?
  // should be same Z as peg1 - assumes bottom+peg height?

  std::cout << "Top Hole Wrld Pose = " << conversion::dumpPoseSimple(wrldHole1) << "\n";

  // Transform from world to robot space
  tf::Pose rbtHole1 = robot._arm->toRobotCoord(wrldHole1);
  std::cout << "Top Hole Robot Coord Pose = " << conversion::dumpPoseSimple(rbtHole1) << "\n";

  // add approximate length of peg to robot pose (otherwise hits into pegboard)
  // rbtHole1 = _robot->addGrasped(rbtHole1); // this adds the grasped object to robot wrist pose (make it higher in Z)
  // std::cout << "Top Hole Robot Coord Pose - Peg = " << conversion::dumpPoseSimple(rbtHole1) << "\n";

  // MOve to retract position above hole
  tf::Pose rHoleApproach = robot._arm->retract(rbtHole1, tf::Vector3(0.0, 0.0, .040));
  std::cout << "Top Hole Robot Coord Retract Pose" << dumpPoseSimple(rHoleApproach) << std::endl;
  //robot.SetAbsoluteSpeed(.01);
  robot.MoveTo(Convert<tf::Pose, robotPose>(rHoleApproach), true);

  // Move to bott of hole and then ungrasp (settool 1.)
  robot.MoveTo(Convert<tf::Pose, robotPose>(rbtHole1), true);

  // robot.SetTool(1.0);
  // tf::Pose holeretract = robot._arm->retract(rbtHole1, tf::Vector3(0.0, 0.0,  .060)); // kludge
  // robot.MoveTo(Convert<tf::Pose, robotPose>(holeretract), true);
  // 1 open 0 closed

  // Read forces touching....
}
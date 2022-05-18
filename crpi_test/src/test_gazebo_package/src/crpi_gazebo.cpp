///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_universal.cpp
//  Revision:        1.0 - 24 June, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Universal Robot UR10 interface definitions.
//`
///////////////////////////////////////////////////////////////////////////////

// won't work - as tf has tf2 embedded in it
//#define TF2_CONVERT_H
//#define TF2_ROS_BUFFER_INTERFACE_H
//#define TF2_ROS_BUFFER_H
#define STRING2(X) #X
#define STRING(x) "WIN VERSION="##STRING2(x)
#pragma message(STRING(_MSC_VER))
#include "conversions.h"
#if 0
// why? only VS2019 IDE?
namespace ros
{
	namespace console
	{
		bool g_initialized;
	}
}
#endif
#include "Hacks.h"
#include <urdf/model.h>

// Boost
#include <boost/format.hpp>
// Error at compile time for non handled convert
#include <boost/static_assert.hpp>

#include "crpi_gazebo.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <fstream>
#include <iostream>
#include <gazebo_plugins2/GetJointsProperties.h>
#include <gazebo_plugins2/GuardedMove.h>
#include <thread>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#ifndef RPY_P_FUZZ
#define RPY_P_FUZZ (0.000001)
#endif
#ifndef SQ
#define SQ(X) (X * X)
#endif
#define VERIFY_MOVING
#define USE_TIMEOUT

//#define NEWTCPIP

//#define UNIVERSAL_NOISY
#define distthresh 0.01f
#define angthresh 7.0f
#define timethresh 40

#include "traj_math.h"
#include <urdf/model.h>
#include "tiny_kdl.h"
using namespace std;
using namespace trajmath;
using namespace crpi_robot;
/*
  joint_state_publisher_topic: /lrmate/joint_states
  gripper_topic: /fanuc_lrmate200id/control
  robot_model_name: lrmate
  use_gazebo: 1

nc:
  baselink: fanuc_base_link
  tiplink: fanuc_link_6

gazebo:
  gzJointPrefix: 'lrmate::fanuc_lrmate200id::'
  gzLeftFingerContactTopic: /gazebo/default/lrmate/fanuc_lrmate200id/motoman_left_finger/left_finger_contact
  gzRightFingerContactTopic: /gazebo/default/lrmate/fanuc_lrmate200id/motoman_right_finger/right_finger_contact
  gzRobotCmdTopicName: /gazebo/default/fanuc/robotcmd
  gzRobotStatusTopicName: /gazebo/default/fanuc/robotstatus
  gzGripperCmdTopicName: /gazebo/default/gripper/fanuc_lrmate200id/control
  gzGripperStatusTopicName: /gazebo/default/gripper/fanuc_lrmate200id/state

  */

using namespace conversion;
ros::AsyncSpinner *CRobotImpl::_spinner;
ros::NodeHandlePtr CRobotImpl::nh;
std::string CRobotImpl::sRosMasterUrl;
std::string CRobotImpl::sRosPackageName;
std::mutex CRobotImpl::robotmutex;

CRobotImpl::CRobotImpl()
{
	bDebug = false;
	bDisplayFT = false;
	bFakeJointStatePublisher = false;
	bJntInited = false;
	bJointStatePublisherUpdate = false;

	kdl = std::shared_ptr<KDL::CTinyKdlSolver>(new KDL::CTinyKdlSolver());

	sRosMasterUrl = getenv("ROS_MASTER_URI");
	sRosMasterUrl = "http://127.0.0.1:11311";
	if (sRosMasterUrl.empty())
		throw std::exception("NO ROS_MASTER_URI");
	sRosPackageName = "CrpiGazebo";

	ros::M_string remappings;
	remappings["__master"] = sRosMasterUrl.c_str(); //
	remappings["__name"] = sRosPackageName;

	// Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
	ros::init(remappings, sRosPackageName);
	ROS_DEBUG("Made it through ros::init");
	try
	{
		nh = ros::NodeHandlePtr(new ros::NodeHandle);
	}
	catch (std::exception e)
	{
		std::cout << e.what();
	}
	ROS_DEBUG("Created ros::NodeHandlePtr");
}

CRobotImpl::~CRobotImpl()
{
	_gzSetJointTrajSrvClient.shutdown();
	_gzGetModelStateSrvClient.shutdown();
	_gzGripperSrvClient.shutdown();
	_gzGetJointPropSrvClient.shutdown();
	ros::shutdown();
}

void CRobotImpl::init(std::string gzGripperTopic)
{
	ROS_DEBUG("Enter CRobotImpl\n");
	try
	{
		// set ROS logging level
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
		{
			ros::console::notifyLoggerLevelsChanged();
		}
		// assign gzGripper service topic 
		nh = ros::NodeHandlePtr(new ros::NodeHandle);
		this->gzGripperTopic = gzGripperTopic; // gazebo ros topic to set joint positions
		_gzSetJointTrajSrvClient = nh->serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

		// gazebo ros topic to get joint positions
		_gzGetModelStateSrvClient = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		//_gzModelState= nh->serviceClient<gazebo_msgs::ModelState>("/gazebo/model_state");

		// advertise ros topic to set joint positions for joint_state_publisher
		if (bJointStatePublisherUpdate)
			_robotJointStatePub = nh->advertise<sensor_msgs::JointState>(this->joint_state_publisher_topic, 1);

		// assign handle for gazebo_msgs::GetJointProperties service client topic name gazebo_msgs::GetJointProperties
		_gzGetJointPropSrvClient = nh->serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

		// assign handle gzRosJointStates service client topic assgined and  is hard coded
		_gzRosJointStatesSrvClient = nh->serviceClient<gazebo_plugins2::GetJointsProperties>("/lrmate/gazebo_ros_joint_state_publisher_service");

		// assign handle to Gz Gripper service client using ROS param topic
		_gzGripperSrvClient = nh->serviceClient<std_srvs::SetBool>(gzGripperTopic);
		if (!_gzGripperSrvClient.exists())
			throw ros::InvalidNameException("Exception: invalid service name " + gzGripperTopic);

		// build KDL chain from WM robot urdf, given base and ee links
		kdl->buildKDLChain(this->robot_urdf, this->robotBaselink, this->robotTiplink);

		// assign handle to F/T sensor subscriber given hard code topic name
		_forceTorqueSensorSub = nh->subscribe("ft_sensor_topic", 1, &CRobotImpl::readForceTorqueSensor, this);

		// assign handle to gaRos guarded more service client - DOES NOT WORK!
		_gzRosGuardedMoveSrvClient = nh->serviceClient<gazebo_plugins2::GuardedMove>("/lrmate/gazebo_ros_guarded_move");

		//_gzJointStateSub= nh->subscribe("joint_states", 1, &CRobotImpl::readJointState, this);
		//  Start ROS async spinner required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
		_spinner = new ros::AsyncSpinner(2);
		_spinner->start();
	}
	catch (ros::InvalidNameException &e)
	{
		ROS_ERROR(format("CRobotImpl::init failed=%s", e.what()).c_str());
	__debugbreak();
	}
}
void CRobotImpl::readJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
	std::unique_lock<std::mutex> lock(robotmutex);
	bJntInited = true;
	currentJnts = *_msg;
}
void CRobotImpl::readForceTorqueSensor(const geometry_msgs::WrenchStampedConstPtr &_msg)
{
	ft = _msg->wrench;
	if (bDisplayFT || ft.force.z > 0.001)
	{
		//std::cout << "Received F/T msg: " << std::endl;
		//std::cout << "Force X=" << ft.force.x;
		//std::cout << "Force Y=" << ft.force.y;
		//std::cout << "Force Z=" << ft.force.z<< std::endl;
	}
}

bool CRobotImpl::isRunning()
{
	return ros::master::check();
}

int CRobotImpl::setGripper(double d)
{
	if (d == 0.0)
	{
		std_srvs::SetBool srv;
		srv.request.data = 1;
		if (!_gzGripperSrvClient.call(srv))
		{
			ROS_ERROR(format("Failed to call service %s close", gzGripperTopic.c_str()).c_str());
			__debugbreak();
			return 1;
		}
	}
	else
	{
		std_srvs::SetBool srv;
		srv.request.data = 0;
		if (!_gzGripperSrvClient.call(srv))
		{
			ROS_ERROR(format("Failed to call service %s open", gzGripperTopic.c_str()).c_str());
			__debugbreak();
			return 1;
		}
	}
	return 0;
}
/////////////////////////////////////////////
tf::Pose CRobotImpl::removeGripper(tf::Pose pose)
{
	if (bDebug)
		std::cout << "With gripper" << dumpPoseSimple(pose) << std::endl;
	pose = GripperInv * pose;
	if (bDebug)
		std::cout << "Wout gripper" << dumpPoseSimple(pose) << std::endl;
	return pose;
}
tf::Pose CRobotImpl::addGripper(tf::Pose pose)
{
	if (bDebug)
		std::cout << "With gripper" << dumpPoseSimple(pose) << std::endl;
	pose = pose * Gripper;
	if (bDebug)
		std::cout << "Wout gripper" << dumpPoseSimple(pose) << std::endl;
	return pose;
}
/////////////////////////////////////////////
//  remove length of grasped object to robot pose
tf::Pose CRobotImpl::removeGrasped(tf::Pose pose)
{
	if (bDebug)
		std::cout << "With grasped peg" << dumpPoseSimple(pose) << std::endl;
	pose =  GraspedInv * pose; 
	if (bDebug)
		std::cout << "Wout grasped peg" << dumpPoseSimple(pose) << std::endl;
	return pose;
}
// adding length of grasped object as away from goal offset to robot pose
tf::Pose CRobotImpl::addGrasped(tf::Pose pose)
{
	if (bDebug)
		std::cout << "Wout grasped peg" << dumpPoseSimple(pose) << std::endl;
	pose = Grasped * pose ;
	if (bDebug)
		std::cout << "With grasped peg" << dumpPoseSimple(pose) << std::endl;
	return pose;
}

/////////////////////////////////////////////
tf::Pose CRobotImpl::retract(tf::Pose pose, tf::Vector3 v)
{
	tf::Pose pRetract(tf::Quaternion(0, 0, 0, 1), v);
	tf::Pose p = pRetract * pose;
	return p;
}

tf::Pose CRobotImpl::toRobotCoord(tf::Pose pose)
{
	if (bDebug)
	{
		std::cout << "toRobotCoord World pose" << dumpPoseSimple(pose) << std::endl;
		std::cout << "toRobotCoord Base" << conversion::dumpPoseSimple(basePose) << std::endl;
		std::cout << "toRobotCoord BaseInverse" << conversion::dumpPoseSimple(basePoseInverse) << std::endl;

	}
	pose = this->basePoseInverse * pose;
	if (bDebug)
		std::cout << "toRobotCoord Robot pose" << dumpPoseSimple(pose) << std::endl;
	return pose;
}
tf::Pose CRobotImpl::toWorldCoord(tf::Pose pose)
{
	if (bDebug)
		std::cout << "With base" << dumpPoseSimple(pose) << std::endl;
	pose = pose * this->basePose;
	if (bDebug)
		std::cout << "Wout base" << dumpPoseSimple(pose) << std::endl;
	return pose;
}
int CRobotImpl::Dwell(double seconds)
{
	//FIXME: if seconds < 0.0001 is zero sleep should be sleep or yield
	std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000.)));
	return 0;
}

void CRobotImpl::publishFakeJoints(trajectory_msgs::JointTrajectoryPoint pt)
{
	std::vector<double> jts = pt.positions;

	// If no one listening don't publish
	if (_robotJointStatePub.getNumSubscribers() == 0)
	{
		std::cerr << "crclimpl::publishFakeJoints NO ONE LISTENING?!?!\n";
		std::cerr << "Check that joint_state_publisher and _robotJointState topic names match EXACTLY\n";
	}

	sensor_msgs::JointState jntCmdMsg;
	const std::vector<std::string> &joint_names = this->jointNames;
	jntCmdMsg.name.insert(jntCmdMsg.name.begin(), joint_names.begin(), joint_names.end());
	assert(joint_names.size() == jts.size());
	jntCmdMsg.position = jts;
	jntCmdMsg.header.stamp = ros::Time::now();
	_robotJointStatePub.publish(jntCmdMsg);
}
tf::Pose CRobotImpl::FK(std::vector<double> positions)
{
	// FIXME: determine what joints names are actually moved
	sensor_msgs::JointState jnts;
	tf::Pose pose;
	pose = kdl->FK(positions);
	return pose;
}
std::vector<double> CRobotImpl::IK(tf::Pose pose, std::vector<double> seed)
{
	std::vector<double> joint_values = kdl->IK(seed, pose);
	return joint_values;
}
std::vector<double> CRobotImpl::IK(tf::Pose pose)
{
	std::vector<double> joint_values; // Fixme get current values for KDL
	joint_values = kdl->IK(this->currentRobotJoints(), pose);
	return joint_values;
}
tf::Pose CRobotImpl::currentPose()
{
	// current pose in robot coordinate space
	tf::Pose nowpose = this->FK(currentRobotJoints());
#if 0
	tf::Pose nowpose;
	gazebo_msgs::GetModelState ms;
	ms.request.model_name=this->_gzrobot_name;

	if(_gzGetModelState.call(ms))
	{
		nowpose=conversion::Convert<geometry_msgs::Pose, tf::Pose>(ms.response.pose);
	}
	std::cout << "Model state "  << dumpPoseSimple(nowpose) << "\n";
#endif
	return nowpose;
}
std::vector<double> CRobotImpl::currentRobotJoints()
{

	std::unique_lock<std::mutex> lock(robotmutex);
	std::vector<double> group_variable_values;

#if 1
	// too much latency using joint_state updates
	// while(!bJntInited && ros::ok())
	// {
	// 	ros::spinOnce();
	// }
	//group_variable_values=currentJnts.position;
	//group_variable_values.resize(jointNames.size());
	//#elif DEFAULT_GETJOINTS

	//FIXME: lots of service calls. But for now...
	gazebo_msgs::GetJointProperties getjointprop;
	//_gzGetJointProp
	for (size_t i = 0; i < jointNames.size(); i++)
	{
		getjointprop.request.joint_name = this->_gzrobot_name + "::" + jointNames[i];
		if (_gzGetJointPropSrvClient.call(getjointprop))
		{
			std::vector<double> pos = getjointprop.response.position;
			group_variable_values.push_back(pos[0]);
		}
		else
		{
		}
	}
#else
	//FIXME: lots of service calls. But for now...
	gazebo_plugins2::GetJointsProperties getjoints;
	getjoints.request.model_name = this->_gzrobot_name;
	std::vector<double> zero_velocity(jointNames.size(), 0.0);
	std::vector<double> group_variable_velocities;
	while (_gzRosJointStatesSrvClient.call(getjoints))
	{
		group_variable_values = getjoints.response.position;
		group_variable_velocities = getjoints.response.velocity;
		group_variable_values.resize(jointNames.size());
		group_variable_velocities.resize(jointNames.size());

		if (std::equal(group_variable_velocities.begin(), group_variable_velocities.end(), zero_velocity.begin()))
			break;
		ros::Duration(0.1).sleep();
	}
	if (!getjoints.response.success)
		std::cerr << "CRobotImpl::currentRobotJoints() _gzRosJointStatesSrvClient error\n";

#endif
	return group_variable_values;
}
moveit_msgs::RobotTrajectory CRobotImpl::calcJointMove(std::vector<double> positions,
													   std::vector<std::string> jointnames,
													   double velscale)
{
	// FIXME: determine what joints names are actually moved assume all
	sensor_msgs::JointState jnts;
	//    geometry_msgs::Pose p ;
	moveit_msgs::RobotTrajectory trajectory;

	try
	{
		trajectory.joint_trajectory.header.frame_id = "/world";
		trajectory.joint_trajectory.header.seq = 0;
		trajectory.joint_trajectory.header.stamp = ros::Time::now();
		trajectory.joint_trajectory.joint_names = jointNames;

		std::vector<double> jnts = currentRobotJoints();

		for (double dIncrement = 0.0; dIncrement < 1.0;)
		{
			// Change from straight line to s curve
			double t = S_Curve::scurve(dIncrement);

			::trajectory_msgs::JointTrajectoryPoint pt;
			for (size_t i = 0; i < jnts.size(); i++)
				pt.positions.push_back(lerp(jnts[i], positions[i], t));
			// fixme: differentiate vel, acc values t-1, t-2
			pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

			// simple test to see if IK worked. Should get all joint positions
			if (pt.positions.size() == trajectory.joint_trajectory.joint_names.size())
				trajectory.joint_trajectory.points.push_back(pt);

			dIncrement = dIncrement + 0.1 * velscale;
		}
	}
	catch (std::exception &e)
	{
		std::cerr << "crclimpl::moveJoints exception " << e.what();
	}

	return trajectory;
}
moveit_msgs::RobotTrajectory CRobotImpl::calcTraj(tf::Pose curpose,
												  tf::Pose rbtFinal,
												  double velscale)
{
	moveit_msgs::RobotTrajectory trajectory;
	std::vector<geometry_msgs::Pose> waypoints;
	try
	{
		// THis is robot coordinate space without gripper
		if (bDebug)
		{
			std::cout << "CRobotImpl::calcMoveTo DestPose = " << conversion::dumpPoseSimple(rbtFinal) << "\n";
			std::cout << "CRobotImpl::calcMoveTo CurPose = " << conversion::dumpPoseSimple(curpose) << "\n";
		}
		geometry_msgs::Pose final_pose = Convert<tf::Pose, geometry_msgs::Pose>(rbtFinal);
		//std::vector<double> jts = move_group->getCurrentJointValues()

		// handle waypoints - where is coordinate frame
		trajectory.joint_trajectory.header.frame_id = "/world";
		trajectory.joint_trajectory.header.seq = 0;
		trajectory.joint_trajectory.header.stamp = ros::Time::now();
		trajectory.joint_trajectory.joint_names = jointNames;

		std::vector<double> seed=this->currentRobotJoints();

		// Did not matter
		for (double dIncrement = 0.0; dIncrement < 1.0;)
		{
			// Change from straight line to s curve
			double t = S_Curve::scurve(dIncrement);
			tf::Pose waypoint;
			waypoint.setOrigin(curpose.getOrigin().lerp(rbtFinal.getOrigin(), t));
			waypoint.setRotation(curpose.getRotation().slerp(rbtFinal.getRotation(), t));
			geometry_msgs::Pose final_waypoint = Convert<tf::Pose, geometry_msgs::Pose>(waypoint);
			//waypoints.push_back(final_waypoint);
			::trajectory_msgs::JointTrajectoryPoint pt;

			// fixme: differentiate vel, acc values t-1, t-2
			pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			pt.positions = this->IK(waypoint, seed);
			seed=pt.positions;

			// simple test to see if IK worked. Should get all joint positions
			if (pt.positions.size() == trajectory.joint_trajectory.joint_names.size())
				trajectory.joint_trajectory.points.push_back(pt);

			dIncrement = dIncrement + 0.1 * velscale;
		}
	}
	catch (std::exception &e)
	{
		ROS_FATAL_STREAM("climpl::moveTo exception " << e.what());
	}
	catch (...)
	{
		ROS_FATAL_STREAM("climpl::moveTo exception ");
	}
	return trajectory;
}
moveit_msgs::RobotTrajectory CRobotImpl::calcMoveTo(tf::Pose rbtFinal,
													double velscale)
{
	return CRobotImpl::calcTraj(currentPose(), rbtFinal, velscale);
}
void CRobotImpl::home()
{
	// HOME Reset robot to home
	trajectory_msgs::JointTrajectoryPoint pt;
	std::vector<double> jts = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	pt.positions = jts;
	gzMovePt(pt);
}

CanonReturn CRobotImpl::moveTraj(tf::Pose rbtCurpose,
						 tf::Pose rbtFinal,
						 double velscale)
{
	CanonReturn hr;
	moveit_msgs::RobotTrajectory traj = calcTraj(rbtCurpose, rbtFinal, velscale);

	// We assume the traj already has time spaced trajectory points
	for(size_t i=0; i< traj.joint_trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint pt = traj.joint_trajectory.points[i];
		if ((hr = gzMovePt(pt)) != CANON_SUCCESS)
			return hr;

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		ros::spinOnce(); // we have to get the Gz command to be serviced
	}
	return CANON_SUCCESS;
}

//http://gazebosim.org/tutorials/?tut=ros_control

// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/8612e61fc4183906f3450a560057b1893a7ade72/gazebo_plugins/src/gazebo_ros_joint_trajectory.cpp
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/37726f6b3c51f6c20dad9834d7741c64811b7b10/.ros1_unported/gazebo_plugins/include/gazebo_plugins/gazebo_ros_joint_state_publisher.h
CanonReturn CRobotImpl::gzMovePt(trajectory_msgs::JointTrajectoryPoint pt)
{
	gazebo_msgs::SetModelConfiguration traj;
	traj.request.model_name = this->_gzrobot_name; //lrmate
	traj.request.urdf_param_name = "";
	traj.request.joint_names = this->jointNames;
	for (size_t k = 0; k < jointNames.size(); k++)
		traj.request.joint_names[k] = this->_gzrobot_name + "::" + traj.request.joint_names[k];
	for (size_t k = 0; k < pt.positions.size(); ++k)
		traj.request.joint_positions.push_back(pt.positions[k]);

	if (bDebug)
		std::cout << "gzMovePt Joints=" << vectorDump<double>(pt.positions) << "\n";

	if (!_gzSetJointTrajSrvClient.call(traj))
	{
		ROS_ERROR("Failed to call service /gazebo/set_model_configuration\n");
		__debugbreak();
		return CANON_FAILURE;
	}
	return CANON_SUCCESS;
}

/**
		 * @brief move "pysically" move the robot given the trajectory.
		 * Of not, only simulation USING GAZEBO is done - using the
		 * move_grpou joints theses are snet to the gazel model containing
		 * these joints as defined initially by the move_group, and these
		 * joints are also updated by sending a command to joint_state_publisher
		 * which is spawned with tthe ROS URDF parameter set - so if multiple
		 * robots are defined, there can be multiple publishers to the  joint_state_publisher
		 * ros node, WHICH HAS NOT BEEN TESTED.  The hope is that there can be TWO OR MORE
		 * move_it trajectory SHIMDS to command different robots in gazebo (using one URDF
		 * to define the multiple robots) that each has its own app to commmand and control
		 * these gazebo robots based on CRCL API (which is synchronous to the crcl robot
		 * app - which may be a problem).
		 */
CanonReturn CRobotImpl::move(moveit_msgs::RobotTrajectory trajectory)
{
	try
	{
		trajectory_msgs::JointTrajectoryPoint pt;

		for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
		{
			pt = trajectory.joint_trajectory.points[i];

			// publish joints to joint state publisher using renamed topic
			if (bFakeJointStatePublisher)
				publishFakeJoints(pt);

			// Publish joints to gazebo
			gzMovePt(pt);
		}

		//if(bDebug)
		//ROS_INFO("%s", currentState().c_str());
	}
	catch (std::exception &e)
	{
		ROS_ERROR_STREAM("CRobotImpl::move exception " << e.what());
			__debugbreak();
		return CANON_FAILURE;
	}
	return CANON_SUCCESS;
}

CWm::CWm(std::string name,
		 tf::Pose pose)
{
	this->_name = name;
	this->_location = pose;
}
CWm::~CWm()
{
	stop();
}
void CWm::stop()
{
	_gzWorldModel.shutdown();
}

void CWm::init(ros::NodeHandlePtr nh, std::shared_ptr<CRobotImpl> robot)
{
	_nh = nh;
	_robot = robot;
}
void CWm::start()
{
	// Subscribes _gzWorldModel to reads all the models in gazebo and reports their poses.
	_gzWorldModel = _nh->subscribe("/gazebo/model_states", 1, &CWm::gzModelStatesCallback, this);
}

void CWm::readConfig()
{

	parameters["gazebo.modeltopicname"] = "/gazebo/default/ariac/model";
	parameters["gazebo.gzJointPrefix"] = "lrmate::fanuc_lrmate200id::";
	parameters["robot.baselink"] = "fanuc_base_link";
	parameters["robot.tiplink"] = "fanuc_link_6";
	parameters["gazebo.gripper_topic"] = "/fanuc_lrmate200id/control";
	parameters["gazebo.robot_model_name"] = "lrmate";
	parameters["joint_state_publisher_topic"] = "/lrmate/joint_states";
	bool bParam = 1;

	/// ROBOT configuration
	////////////////////////////////////////////
	bParam &= _nh->getParam(ns + "/lrmate/tiplink", _robot->robotTiplink);
	bParam &= _nh->getParam(ns + "/lrmate/baselink", _robot->robotBaselink);

	// Read the robot urdf XML string
	bParam &= _nh->getParam(ns + "/lrmate/robot_description", _robot->robot_urdf);
	if (!bParam)
	{
		ROS_ERROR("rcs_robot_type::configure getParam failed");
		__debugbreak();
	}

	ROS_DEBUG("/lrmate/tiplink=%s", _robot->robotTiplink);
	ROS_DEBUG("/lrmate/baselink=%s", _robot->robotBaselink);
	ROS_DEBUG("/lrmate/robot_description=%s", _robot->robot_urdf);

	_robot->joint_state_publisher_topic = parameters["joint_state_publisher_topic"];

	std::vector<double> deeoffset;
	//_nh->getParam(ns + "/xform/gripper", deeoffset);
	//_robot->Gripper = Convert<std::vector<double>, tf::Pose>(deeoffset);

	_robot->_gzrobot_name = parameters["gazebo.robot_model_name"];
	if (!this->parseURDF())
		ROS_DEBUG("CWm::ParseURDF failed");
}
std::string CWm::getParameter(std::string name, std::string defaultStr)
{
	std::map<std::string, std::string>::iterator it;
	if ((it = parameters.find(name)) != parameters.end())
		return (*it).second;
	return defaultStr;
}
bool CWm::getInstance(std::string name, CWm &wm)
{
	std::unique_lock<std::mutex> lock(wmmutex);
	auto it = find_if(instances.begin(), instances.end(), [&name](const CWm &obj)
					  { return obj._name == name; });
	if (it != instances.end())
	{
		wm = (*it);
		return true;
	}
	return false;
}
std::string CWm::toString()
{
	std::stringstream ss;
	std::unique_lock<std::mutex> lock(wmmutex);
	for(size_t i=0; i< instances.size(); i++)
	{
		CWm &wm (instances[i]);
		ss << wm._name << "=" << dumpPoseRPY(wm._location)<< "\n";
		//instances.push_back(CWm(name, centroid))
	}

	return ss.str();
}

void CWm::storeInstance(std::string name,
						tf::Pose centroid,
						tf::Vector3 scale)
{

	std::unique_lock<std::mutex> lock(wmmutex);

	// Find out if instance has already been defined
	auto it = find_if(instances.begin(), instances.end(), [&name](const CWm &obj)
					  { return obj._name == name; });
	size_t index;
	if (it != instances.end())
	{
		index = std::distance(instances.begin(), it);
		bReadAllInstances = true;
	}
	else
	{
		std::string type(name);
		instances.push_back(CWm(name, centroid));
		index = instances.size() - 1;
	}

	instances[index]._location = centroid;
}
void CWm::gzModelStatesCallback(const gazebo_msgs::ModelStates &gzstate_current)
{
	static int nTest = 0;
	ROS_DEBUG_THROTTLE(60, "gzModelStatesCallback");
	for (size_t i = 0; i < gzstate_current.name.size(); i++)
	{
		std::string name = gzstate_current.name[i];
		tf::Pose pose = Convert<geometry_msgs::Pose, tf::Pose>(gzstate_current.pose[i]);
		tf::Vector3 scale(1., 1., 1.);

		// Lock updates of instance and model link to id
		{
			storeInstance(name, pose, scale);
		}
	}
}
bool CWm::parseURDF()
{
	if (_robot->robot_urdf.empty())
	{
		ROS_FATAL(" rcs_robot_type::parseURDF() empty robot description");
		return false;
	}

	if (!robot_model.initString(_robot->robot_urdf))
	{
		ROS_FATAL(" robot_model.initString(robot_urdf) FAILED");
	}

	_robot->robotName = robot_model.getName();

	// These vectdors are cleared in case parseURDF is called twice...
	_robot->linkNames.clear();
	_robot->jointNames.clear();
	_robot->axis.clear();
	_robot->xyzorigin.clear();
	_robot->rpyorigin.clear();
	_robot->jointHasLimits.clear();
	_robot->jointMin.clear();
	_robot->jointMax.clear();
	_robot->jointEffort.clear();
	_robot->jointVelmax.clear();

	urdf::LinkConstSharedPtr link = robot_model.getLink(_robot->robotTiplink);
	while (link->name != _robot->robotBaselink)
	{ // && joint_names.size() <= num_joints_) {
		_robot->linkNames.push_back(link->name);
		urdf::JointSharedPtr joint = link->parent_joint;
		if (joint)
		{
			if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
			{

				_robot->jointNames.push_back(joint->name);
				_robot->axis.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
				_robot->xyzorigin.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
				double roll, pitch, yaw;
				joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
				_robot->rpyorigin.push_back(tf::Vector3(roll, pitch, yaw));

				float lower, upper, maxvel = 0.0, maxeffort = 0.0;
				int hasLimits;
				if (joint->type != urdf::Joint::CONTINUOUS)
				{
					maxvel = joint->limits->velocity;
					maxeffort = joint->limits->effort;
					if (joint->safety)
					{
						lower = joint->safety->soft_lower_limit;
						upper = joint->safety->soft_upper_limit;
					}
					else
					{
						lower = joint->limits->lower;
						upper = joint->limits->upper;
					}
					hasLimits = 1;
				}
				else
				{
					lower = -M_PI;
					upper = M_PI;
					hasLimits = 0;
				}
				if (hasLimits)
				{
					_robot->jointHasLimits.push_back(true);
					_robot->jointMin.push_back(lower);
					_robot->jointMax.push_back(upper);
				}
				else
				{
					_robot->jointHasLimits.push_back(false);
					_robot->jointMin.push_back(-M_PI);
					_robot->jointMax.push_back(M_PI);
				}
				_robot->jointEffort.push_back(maxeffort);
				_robot->jointVelmax.push_back(maxvel);
			}
		}
		else
		{
			ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
		}
		link = link->getParent();
	}

	std::reverse(_robot->linkNames.begin(), _robot->linkNames.end());
	std::reverse(_robot->jointNames.begin(), _robot->jointNames.end());
	std::reverse(_robot->jointMin.begin(), _robot->jointMin.end());
	std::reverse(_robot->jointMax.begin(), _robot->jointMax.end());
	std::reverse(_robot->jointHasLimits.begin(), _robot->jointHasLimits.end());
	std::reverse(_robot->axis.begin(), _robot->axis.end());
	std::reverse(_robot->xyzorigin.begin(), _robot->xyzorigin.end());
	std::reverse(_robot->rpyorigin.begin(), _robot->rpyorigin.end());
	std::reverse(_robot->jointEffort.begin(), _robot->jointEffort.end());
	std::reverse(_robot->jointVelmax.begin(), _robot->jointVelmax.end());

	return true;
}

std::mutex CWm::wmmutex;
std::vector<CWm> CWm::instances;
/// Parameters contains the rosparam definitions
std::map<std::string, std::string> CWm::parameters;
bool CWm::bReadAllInstances;

//	static void feedbackThread(void* param)
//	{
//		static crpi_timer timer;
//		gazeboHandler* uH = (gazeboHandler*)param;
//		bool connected = false;
//		char* buffer;
//		int get;
//		robotPose pose;
//		robotAxes axes;
//		robotPose force;
//		robotPose speed;
//		robotIO io;
//
//		buffer = new char[1044];
//
//		ulapi_integer client = 0;
//		while (uH->runThread)
//		{
//
//			if (client > 0)
//			{
//
//				if (get == 812 || get == 1044)
//				{
//					//! Parse feedback from robot
//		   //         if (parseFeedback(get, buffer, pose, axes, io, force, speed))
//					{
//						uH->mtx.lock();
//
//						//! Store feedback from robot
//						uH->curPose = pose;
//						uH->poseGood = true;
//						uH->curAxes = axes;
//						uH->curForces = force;
//						uH->curSpeeds = speed;
//						uH->curIO = io;
//						uH->mtx.unlock();
//
//						//cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.xrot << ", " << pose.yrot << ", " << pose.zrot << ")" << endl;
//						//cout << "(" << axes.axis.at(0) << ", " << axes.axis.at(1) << ", " << axes.axis.at(2) << ", " << axes.axis.at(3) << ", " << axes.axis.at(4) << ", " << axes.axis.at(5) << ")" << endl;
//					}
//				} // if (get == 812 || 1044)
//				else
//				{
//					//! Error reading
//#ifdef KEEP_CONNECTION
////          cout << "Bad read.  Restart connection." << endl;
//					ulapi_socket_close(client);
//					client = 0;
//#endif
//
//				}
//			} // if (client > 0)
//
//			std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(33.)));
//
//		} // while (uH->runThread)
//		cout << "Quitting thread" << endl;
//		delete[] buffer;
//		return;
//	}

LIBRARY_API CrpiGazebo::CrpiGazebo(CrpiRobotParams &params) : firstIO_(true)
{
	bDebug = false;
	double Xtheta, Ytheta, Ztheta;
	params_ = params;
	handle_.params = params_;

	maxSpeed_ = 1.0f;
	maxAccel_ = 3.0f;

	//! These are nominal values defined by trial and error.  Use the SetAbsoluteSpeed and
	//! SetAbsoluteAcceleration commands to override
	speed_ = 1.0f;
	acceleration_ = 0.2f;

	mssgBuffer_ = new char[8192];
	//ulapi_init();

	angleUnits_ = RADIAN;
	lengthUnits_ = METER;
	for (int i = 0; i < 6; ++i)
	{
		axialUnits_[i] = RADIAN;
	}

	//task = ulapi_task_new();
	//		handle_.TCPIPhandle = ulapi_mutex_new(17);
	handle_.rob = this;
	handle_.runThread = true;
	handle_.poseGood = false;
	handle_.curTool = -1;

	pin_ = new matrix(3, 1);
	pout_ = new matrix(3, 1);

	forward_ = new matrix(4, 4);
	backward_ = new matrix(4, 4);
	matrix r(3, 3);

	Xtheta = params_.mounting->xrot * (3.141592654 / 180.0f);
	Ytheta = params_.mounting->yrot * (3.141592654 / 180.0f);
	Ztheta = params_.mounting->zrot * (3.141592654 / 180.0f);

	vector<double> vtemp;
	vtemp.push_back(Xtheta);
	vtemp.push_back(Ytheta);
	vtemp.push_back(Ztheta);

	r.rotEulerMatrixConvert(vtemp);
	//! Copy rotation matrix to homogeneous transformation matrix
	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			forward_->at(x, y) = r.at(x, y);
		}
	}
	forward_->at(0, 3) = params_.mounting->x;
	forward_->at(1, 3) = params_.mounting->y;
	forward_->at(2, 3) = params_.mounting->z;
	forward_->at(3, 3) = 1.0f;
	*backward_ = forward_->inv(); //JAM forward_->matrixInv(*forward_, *backward_); //
}

LIBRARY_API CrpiGazebo::~CrpiGazebo()
{
	handle_.runThread = false;
	delete forward_;
	delete backward_;
	delete pin_;
	delete pout_;
}

LIBRARY_API CanonReturn CrpiGazebo::ApplyCartesianForceTorque(robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator)
{
	//! TODO
	return CANON_FAILURE;
}

LIBRARY_API CanonReturn CrpiGazebo::ApplyJointTorque(robotAxes &robotJointTorque)
{
	//! TODO
	return CANON_FAILURE;
}

/***
 * init performs CRPI pionter assignment operations, and reads
 * the configuration from the ROS param server using nh and the
 * given robot arm.
 * assign a new CRobotimpl to the _arm pointer 
 * assign a wm to the _wm pointer
 * initialize the wm to contain the assigned nh and_arm 
 * readConfig reads ROS param server  and parses robot URDF into WM _robot (arm)
 * Arm init:
 * 	 set ROS logging level
*	 assign gzGripper service topic 
*	 gazebo ros topic to get joint positions
*	 advertise ros topic to set joint positions for joint_state_publisher
*	 assign handle for gazebo_msgs::GetJointProperties service client topic name gazebo_msgs::GetJointProperties
*	 assign handle gzRosJointStates service client topic assgined and  is hard coded
*	 assign handle to Gz Gripper service client using ROS param topic
*	 build KDL chain from WM robot urdf, given base and ee links
*	 assign handle to F/T sensor subscriber given hard code topic name
*	 assign handle to gaRos guarded more service client - DOES NOT WORK!
*	 Start ROS async spinner required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
* Start wm thread:
*	Subscribes _gzWorldModel ahdle to reads all the models in gazebo and reports their poses.
 */
void CrpiGazebo::init()
{

	_arm = std::shared_ptr<CRobotImpl>(new CRobotImpl);
	_wm = std::shared_ptr<CWm>(new CWm());
	_wm->init(CRobotImpl::nh, _arm);
	_wm->readConfig();
	_arm->init(_wm->getParameter("gazebo.gripper_topic", "/fanuc_lrmate200id/control"));

	ROS_DEBUG("Finish CrpiGazebo::init()\n");
	_wm->start();
}
void CrpiGazebo::start()
{
	//		ulapi_task_start((ulapi_task_struct*)task, feedbackThread, &handle_, ulapi_prio_lowest(), 0);
	while (handle_.poseGood != true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(100.0)));
	}
}
LIBRARY_API CanonReturn CrpiGazebo::SetTool(double percent)
{
	// Either open/closed sorry - force control gripping.
	if (percent < 0.05)
		_arm->setGripper(0.0);
	else
		_arm->setGripper(1.0);

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::Couple(const char *targetID)
{
	return CANON_FAILURE;
}

LIBRARY_API CanonReturn CrpiGazebo::Message(const char *message)
{
	return CANON_FAILURE;
}

LIBRARY_API CanonReturn CrpiGazebo::MoveStraightTo(robotPose &pose, bool useBlocking)
{
	return this->MoveTo(pose, useBlocking);
}

LIBRARY_API CanonReturn CrpiGazebo::MoveThroughTo(robotPose *poses,
												  int numPoses,
												  robotPose *accelerations,
												  robotPose *speeds,
												  robotPose *tolerances)
{
	bool status = true;

	//! This is a temporary function definition
	for (int x = 0; x < numPoses; ++x)
	{
		status &= (MoveTo(poses[x], true) == CANON_SUCCESS);
		if (!status)
		{
			//! Error when executing multi move
			return CANON_FAILURE;
		}
	}

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::MoveTo(robotPose &pose, bool useBlocking)
{
	//  Transfrom pose from robot coord to world coord
	// I dont think we have to do this. All robots goals are robot coordinates.
	if (lengthUnits_ == MM)
	{
		pose.x *= 0.001f;
		pose.y *= 0.001f;
		pose.z *= 0.001f;
	}
	else if (lengthUnits_ == INCH)
	{
		pose.x *= 0.0254f;
		pose.y *= 0.0254f;
		pose.z *= 0.0254f;
	}
	if (angleUnits_ == DEGREE)
	{
		pose.xrot *= (3.141592654f / 180.0f);
		pose.yrot *= (3.141592654f / 180.0f);
		pose.zrot *= (3.141592654f / 180.0f);
	}

	// We will  assume robot commands are in robot coordinate space,
	// but they don't have to be (e.g., 2 robot defined in URDF with offsets).
	tf::Pose eeFinal = Convert<robotPose, tf::Pose>(pose);
	if (bDebug)
		std::cout << "CrpiGazebo::MoveTo Pose = " << conversion::dumpPoseSimple(eeFinal) << "\n";

	//moveit_msgs::RobotTrajectory trajectory;
	std::vector<geometry_msgs::Pose> waypoints;
	try
	{
		//We **ASSUME** robot origin final  coordinate space
		tf::Pose rbtOrigin = eeFinal;

		// remove  robot gripper from destination pose
		tf::Pose rbtFinal = rbtOrigin;
		rbtFinal = _arm->removeGripper(rbtOrigin);
		if (bDebug)
			std::cout << "CrpiGazebo::MoveTo Final Pose = " << conversion::dumpPoseSimple(rbtFinal) << "\n";

		moveit_msgs::RobotTrajectory traj = _arm->calcMoveTo(rbtFinal, speed_); // where did velocity scale go?
		if (_arm->bDebug)
			for (size_t i = 0; i < traj.joint_trajectory.points.size(); i++)
			{
				trajectory_msgs::JointTrajectoryPoint pt = traj.joint_trajectory.points[i];
				//std::cout << "Trajectory Joints[" << i << "]=" << vectorDump<double>(traj.joint_trajectory.points[i].positions) << "\n";
				tf::Pose nextpose = _arm->FK(traj.joint_trajectory.points[i].positions);
				std::cout << "Trajectory Pose [" << i << "]=" << conversion::dumpPoseSimple(nextpose) << "\n";
			}
		_arm->move(traj);
	}
	catch (std::exception &e)
	{
		ROS_FATAL_STREAM("climpl::moveTo exception " << e.what());
	}
	catch (...)
	{
		ROS_FATAL_STREAM("climpl::moveTo exception ");
	}
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotAxes(robotAxes *axes)
{
	handle_.mtx.lock();
	handle_.curAxes.axis = _arm->currentRobotJoints();
	*axes = handle_.curAxes;
	handle_.mtx.unlock();

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotForces(robotPose *forces)
{
	robotPose temp;
	matrix pintemp(4, 4), r(3, 3);
	matrix pouttemp(4, 4);

	handle_.mtx.lock();
	*forces = handle_.curForces;
	handle_.mtx.unlock();

	transformFromMount(handle_.curForces, temp, false);

	forces->x = temp.x;
	forces->y = temp.y;
	forces->z = temp.z;
	forces->xrot = temp.xrot;
	forces->yrot = temp.yrot;
	forces->zrot = temp.zrot;

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotIO(robotIO *io)
{
	handle_.mtx.lock();
	*io = handle_.curIO;
	handle_.mtx.unlock();
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotPose(robotPose *pose)
{

	// what does this mean?
	//transformFromMount(handle_.curPose, temp);

	handle_.mtx.lock();
	handle_.curPose = conversion::Convert<tf::Pose, robotPose>(_arm->currentPose());
	*pose = handle_.curPose;
	handle_.mtx.unlock();

	return CANON_SUCCESS;
}
// https://answers.gazebosim.org/questions/
LIBRARY_API CanonReturn CrpiGazebo::GetRobotSpeed(robotPose *speed)
{
	robotPose temp;
	matrix pintemp(4, 4), r(3, 3);
	matrix pouttemp(4, 4);

	handle_.mtx.lock();
	*speed = handle_.curSpeeds;
	handle_.mtx.unlock();

	transformFromMount(handle_.curSpeeds, temp);

	speed->x = temp.x;
	speed->y = temp.y;
	speed->z = temp.z;
	speed->xrot = temp.xrot;
	speed->yrot = temp.yrot;
	speed->zrot = temp.zrot;

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotSpeed(robotAxes *speed)
{

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::GetRobotTorques(robotAxes *torques)
{
	// http://gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors
	handle_.mtx.lock();
	*torques = handle_.curAxes;
	handle_.mtx.unlock();

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::MoveAttractor(robotPose &pose)
{
	robotPose temp, temp2, temp3;
	robotPose curPose;
	vector<double> target;

#if 0
#ifdef UNIVERSAL_NOISY
		cout << "target: " << endl;
		pose.print();
#endif

		GetRobotPose(&curPose);
#ifdef UNIVERSAL_NOISY
		cout << "current: " << endl;
		curPose.print();
#endif

		transformToMount(curPose, temp2);
#ifdef UNIVERSAL_NOISY
		cout << "current at base: " << endl;
		temp2.print();
#endif

		//! Get target pose relative to current pose, these become the X, Y and Z offsets used during force control
		transformToMount(pose, temp3);
#ifdef UNIVERSAL_NOISY
		cout << "target at base: " << endl;
		temp3.print();
#endif
		temp = temp2 - temp3;
#ifdef UNIVERSAL_NOISY
		cout << "offset: " << endl;
		temp.print();
#endif

		target.push_back(temp3.x);
		target.push_back(temp3.y);
		target.push_back(temp3.z);
		target.push_back(temp3.xrot);
		target.push_back(temp3.yrot);
		target.push_back(temp3.zrot);

		if (generateParameter('F', 'E', target))
		{
			if (send())
			{
				//! Okay

		}
			else
			{
				//! Oops
			}
		}
		else
		{
			cout << "Bad force command" << endl;
		}
#endif
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::MoveToAxisTarget(robotAxes &axes, bool useBlocking)
{
	int count = _arm->jointNames.size();
	std::vector<double> jts;
	copy(axes.axis.begin(), axes.axis.begin() + count, back_inserter(jts));
	moveit_msgs::RobotTrajectory traj = _arm->calcJointMove(jts, _arm->jointNames);
	_arm->move(traj);
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetAbsoluteAcceleration(double acceleration)
{
	if (acceleration > maxAccel_ || acceleration < 0.0f)
	{
		return CANON_REJECT;
	}

	acceleration_ = acceleration;
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetAbsoluteSpeed(double speed)
{
	if (speed > maxSpeed_ || speed < 0.0f)
	{
		return CANON_FAILURE;
	}

	speed_ = speed;
	return CANON_SUCCESS;
}

//  ("degree" or "radian")
LIBRARY_API CanonReturn CrpiGazebo::SetAngleUnits(const char *unitName)
{
	if (strcmp(unitName, "degree") == 0)
	{
		angleUnits_ = DEGREE;
	}
	else if (strcmp(unitName, "radian") == 0)
	{
		angleUnits_ = RADIAN;
	}
	else
	{
		CANON_FAILURE;
	}

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetAxialSpeeds(double *speeds)
{
	//! TODO
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetAxialUnits(const char **unitNames)
{
	//! Not yet implemented
	return CANON_REJECT;
}

LIBRARY_API CanonReturn CrpiGazebo::SetEndPoseTolerance(robotPose &tolerances)
{
	//! Not yet implemented
	return CANON_REJECT;
}

LIBRARY_API CanonReturn CrpiGazebo::SetIntermediatePoseTolerance(robotPose *tolerances)
{
	//! Not yet implemented
	return CANON_REJECT;
}

LIBRARY_API CanonReturn CrpiGazebo::SetLengthUnits(const char *unitName)
{
	if (strcmp(unitName, "meter") == 0)
	{
		lengthUnits_ = METER;
	}
	else if (strcmp(unitName, "mm") == 0)
	{
		lengthUnits_ = MM;
	}
	else if (strcmp(unitName, "inch") == 0)
	{
		lengthUnits_ = INCH;
	}
	else
	{
		CANON_FAILURE;
	}

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetParameter(const char *paramName, void *paramVal)
{

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetRelativeAcceleration(double percent)
{
	if (percent > 1.0f || percent < 0.0f)
	{
		return CANON_FAILURE;
	}

	acceleration_ = maxAccel_ * percent;
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetRelativeSpeed(double percent)
{
	if (percent > 1.0f || percent < 0.0f)
	{
		return CANON_FAILURE;
	}

	speed_ = maxSpeed_ * percent;
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetRobotIO(robotIO &io)
{
	bool changed = false;

	//handle_.moveMe.str(string());
	//handle_.moveMe << "def myProg():\n";
	//for (int x = 0; x < io.ndio; ++x)
	//{
	//	if (firstIO_ || (curIO_.dio[x] != io.dio[x]))
	//	{
	//		handle_.moveMe << "set_digital_out(" << x << ", ";
	//		handle_.moveMe << (io.dio[x] ? "True" : "False") << ")\n";
	//		changed = true;
	//	}
	//}
	//handle_.moveMe << "end\n";
	////! Send message to robot
	//if (changed)
	//{
	//	if (!send())
	//	{
	//		//! error sending
	//		return CANON_FAILURE;
	//	}
	//}
	//timer_.waitUntil(10);

	curIO_ = io;
	firstIO_ = false;

	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::SetRobotDO(int dig_out, bool val)
{
#if 0
		if (dig_out < 0 || dig_out >= curIO_.ndio)
		{
			return CANON_REJECT;
		}

		handle_.moveMe.str(string());
		handle_.moveMe << "def myProg():\n";
		handle_.moveMe << "set_digital_out(" << dig_out << ", ";
		handle_.moveMe << (val ? "True" : "False") << ")\n";
		handle_.moveMe << "end\n";
		//! Send message to robot
		if (!send())
		{
			//! error sending
			return CANON_FAILURE;
		}
		curIO_.dio[dig_out] = val;
		firstIO_ = false;
#endif
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::StopMotion(int condition)
{
	handle_.mtx.lock();
	//handle_.moveMe.str(string());

	////! stopl(acceleration)
	//handle_.moveMe << "def myProg():\n";
	//handle_.moveMe << "stopl(3.0)\n";
	//handle_.moveMe << "end\n";
	handle_.mtx.unlock();

	//! Send message to robot
	//if (!send())
	//{
	//	//! error sending
	//	return CANON_FAILURE;
	//}
	return CANON_SUCCESS;
}

LIBRARY_API CanonReturn CrpiGazebo::MoveBase(robotPose &to)
{
	//! Not applicable
	return CANON_REJECT;
}

LIBRARY_API CanonReturn CrpiGazebo::PointHead(robotPose &to)
{
	//! Not applicable
	return CANON_REJECT;
}

//LIBRARY_API CanonReturn CrpiGazebo::PointAppendage(CanonRobotAppendage app_ID,
//	robotPose& to)
//{
//	//! Not applicable
//	return CANON_REJECT;
//}

LIBRARY_API bool CrpiGazebo::generateMove(char moveType, char posType, char deltaType, vector<double> &input)
{
	bool state = true;

	//! Check validity of inputs...
	//!   Check movement type
	state &= (moveType == 'P' || moveType == 'L' || moveType == 'F');
	//!   Check position type
	state &= (posType == 'C' || posType == 'A');
	//!   Check absolute or relative motion
	state &= (deltaType == 'A' || deltaType == 'R');
	//!   Check PTP-only angle movements
	state &= !(posType == 'A' && moveType == 'L');
	//!   Check for proper amount of input arguments
	//state &= (input.size() == 6);

	if (!state)
	{
		//! Invalid arguments generating move
		return false;
	}

	handle_.mtx.lock();
	handle_.moveMe.str(string());

	handle_.moveMe << "def myProg():\n";
	handle_.moveMe << "move" << (moveType == 'P' ? "j(" : "l(");
	handle_.moveMe << (posType == 'C' ? "p[" : "[") << ((deltaType == 'A' ? 0.0f : curPose_[0]) + input.at(0)) << ", "
				   << ((deltaType == 'A' ? 0.0f : curPose_[1]) + input.at(1)) << ", "
				   << ((deltaType == 'A' ? 0.0f : curPose_[2]) + input.at(2)) << ", "
				   << ((deltaType == 'A' ? 0.0f : curPose_[3]) + input.at(3)) << ", "
				   << ((deltaType == 'A' ? 0.0f : curPose_[4]) + input.at(4)) << ", "
				   << ((deltaType == 'A' ? 0.0f : curPose_[5]) + input.at(5))
				   << "]"
				   << ", v=" << speed_ << ")\n";
	handle_.moveMe << "end\n";
	//cout << handle_.moveMe.str().c_str() << endl;
	/*
		handle_.moveMe << "while(True):\nforcem" << (moveType == 'P' ? "j(" : "l(");
		handle_.moveMe << (posType == 'C' ? "p[" : "[") << ((deltaType == 'A' ? 0.0f : curPose_[0]) + input.at(0)) << ", "
					   << ((deltaType == 'A' ? 0.0f : curPose_[1]) + input.at(1)) << ", "
					   << ((deltaType == 'A' ? 0.0f : curPose_[2]) + input.at(2)) << ", "
					   << ((deltaType == 'A' ? 0.0f : curPose_[3]) + input.at(3)) << ", "
					   << ((deltaType == 'A' ? 0.0f : curPose_[4]) + input.at(4)) << ", "
					   << ((deltaType == 'A' ? 0.0f : curPose_[5]) + input.at(5))
					   << "], a=" << acceleration_ << ", v=" << speed_ << ")\nsync()\nend\n";
		*/
	handle_.mtx.unlock();

	return true;
}

LIBRARY_API bool CrpiGazebo::generateTool(char mode, double value)
{
	if (!(mode == 'B' || mode == 'A' || mode == 'D'))
	{
		//! Value must be between 0 and 1
		return false;
	}

	//! TODO:  Populate variables
	//moveMe_.str(string());
	//moveMe_ << "(0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000)";
	return true;
}

LIBRARY_API bool CrpiGazebo::generateParameter(char paramType, char subType, vector<double> &input)
{
	bool state = true;

	//! Check validity of inputs...
	//!   Check movement type
	state &= (paramType == 'A' || paramType == 'S' || paramType == 'F' || paramType == 'T');
	//!   Check position type
	state &= (subType == 'A' || subType == 'R' || subType == 'E' || subType == 'D' || subType == 'C');

	if (paramType == 'F')
	{
		state &= (subType == 'E' || subType == 'D');
	}
	else if (paramType == 'T')
	{
		state &= (subType == 'D' || subType == 'C');
	}
	else
	{
		state &= (subType == 'A' || subType == 'R');
	}

	if (!state)
	{
		//! Invalid arguments generating move
		return false;
	}

	handle_.mtx.lock();
	handle_.moveMe.str(string());
	handle_.moveMe << "def myProg():\n";
	switch (paramType)
	{
	case 'A':
		break;
	case 'S':
		break;
	case 'T':
		if (subType == 'D')
		{
			//! Define TCP
			handle_.moveMe << "set_tcp(p[" << input.at(0) << ", " << input.at(1) << ", " << input.at(2)
						   << ", " << input.at(3) << ", " << input.at(4) << ", " << input.at(5) << "])\n";
		}
		else
		{
			//! Center of mass
			handle_.moveMe << "set_payload(" << input.at(0) << ", (" << input.at(1) << ", " << input.at(2)
						   << ", " << input.at(3) << "))\n";
		}
		break;
	case 'F':
		if (handle_.curTool < 0)
		{
			//! Cannot initiate force control without tool definition
			return false;
		}

		if (subType == 'E')
		{
			handle_.moveMe << "def myFunk():\n";
			//handle_.moveMe << "  set_payload(" << params_.tools.at(handle_.curTool).mass << ", ("
			//               << (params_.tools.at(handle_.curTool).centerMass.x / 1000.0f) << ", "
			//               << (params_.tools.at(handle_.curTool).centerMass.y / 1000.0f) << ", "
			//               << (params_.tools.at(handle_.curTool).centerMass.z / 1000.0f) << "))\n";
			matrix rot(3, 3);
			vector<double> euler;
			euler.push_back(params_.mounting->xrot);
			euler.push_back(params_.mounting->yrot);
			euler.push_back(params_.mounting->zrot);
			rot.rotEulerMatrixConvert(euler);
			//handle_.moveMe << "  set_gravity([" << (9.82f * rot.at(2, 0)) << ", " << (9.82f * rot.at(2, 1)) << ", "
			//               << (9.82f * rot.at(2, 2)) << "])\n";
			handle_.moveMe << "  thread Force_properties_calculation_thread_1():\n";
			handle_.moveMe << "    while (True):\n";
			//handle_.moveMe << "      force_mode(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], [0.0, 0.0, 30.0, 0.0, 0.0, 0.0], 3, [0.15, 0.15, 0.25, 0.17, 0.17, 0.17])\n";
			handle_.moveMe << "      force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0.0, 0.0, 30.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.2, 0.17453292519943295, 0.17453292519943295, 0.17453292519943295])\n";
			//        handle_.moveMe << "      force_mode(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], [0.0, 0.0, 20.0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.015, 0.17453292519943295, 0.17453292519943295, 0.17453292519943295])\n";
			handle_.moveMe << "      sync()\n";
			handle_.moveMe << "    end\n";
			handle_.moveMe << "  end\n";
			handle_.moveMe << "  global thread_handler_1 = run Force_properties_calculation_thread_1()\n";

			//handle_.moveMe << "  global cloud = tool_pose()\n";// p[0.0000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000]\n";
			handle_.moveMe << "  global X = " << input.at(0) << "\n";
			handle_.moveMe << "  global Y = " << input.at(1) << "\n";
			handle_.moveMe << "  global Z = " << input.at(2) << "\n";
			handle_.moveMe << "  global XR = " << input.at(3) << "\n";
			handle_.moveMe << "  global YR = " << input.at(4) << "\n";
			handle_.moveMe << "  global ZR = " << input.at(5) << "\n";

			//! Enable force mode
			if (input.size() == 6)
			{
				//handle_.moveMe << "  global pt = pose_trans(cloud, p[X,Y,Z,0,0,0])\n";
				//          handle_.moveMe << "    movel(pose_trans(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], pt), a=1.39, v=1.04)\n";
				handle_.moveMe << "  movej(p[X, Y, Z, XR, YR, ZR], a=0.39, v=0.25)\n";
				//          handle_.moveMe << "    movel(p[" << input.at(0) << ", " << input.at(1) << ", " << input.at(2) << ", " << input.at(3)
				//                         << ", " << input.at(4) << ", " << input.at(5) << "], a=0.39, v=0.25)\n";
			}
			handle_.moveMe << "  kill thread_handler_1\n";
			//handle_.moveMe << "    end_force_mode()\n";
			//      handle_.moveMe << "  end\n"; //! "Walk()"
			//! Program begins here
			//        handle_.moveMe << "  Walk()\n";
		}
		else
		{
			//! Disable force mode
			handle_.moveMe << "  end_force_mode()\n";
		}
		handle_.moveMe << "end\n";

		break;
	default:
		//! Shouldn't get here.  We checked for this already.
		break;
	}
	handle_.moveMe << "end\n";

	handle_.mtx.unlock();

	return true;
}

//	LIBRARY_API bool CrpiGazebo::send()
//	{
//#ifndef NEWTCPIP
//		handle_.mtx.lock();
//		ulapi_integer client = ulapi_socket_get_client_id(handle_.params.tcp_ip_port, handle_.params.tcp_ip_addr);
//		ulapi_socket_set_nonblocking(client);
//		handle_.mtx.unlock();
//#endif
//
//		int sent;
//#ifndef NEWTCPIP
//		if (client > 0)
//#else
//		if (handle_.clientID > 0)
//#endif
//		{
//			handle_.mtx.lock();
//#ifdef UNIVERSAL_NOISY
//			cout << handle_.moveMe.str().c_str() << endl;
//#endif
//
//
//			//cout << handle_.moveMe.str().c_str() << endl;
//#ifndef NEWTCPIP
//			sent = ulapi_socket_write(client, handle_.moveMe.str().c_str(), strlen(handle_.moveMe.str().c_str()) + 1);
//#else
//			sent = ulapi_socket_write(handle_.clientID, handle_.moveMe.str().c_str(), strlen(handle_.moveMe.str().c_str()) + 1);
//#endif
//
//			handle_.mtx.unlock();
//		}
//		else
//		{
//			cout << endl << "cannot connect" << endl;
//			return false;
//		}
//#ifndef NEWTCPIP
//		ulapi_socket_close(client);
//#endif
//
//		return true;
//	}

//LIBRARY_API bool CrpiGazebo::get()
//{
//	int x = 0;
//	if (serialUsed_)
//	{
//		//      printf ("getting feedback...\n");
//		x = ulapi_serial_read(serialID_, mssgBuffer_, 8192);
//		//      printf ("%d read\n", x);
//		return true;
//	}
//	else
//	{
//		//! TODO
//		return false;
//	}
//}

LIBRARY_API bool CrpiGazebo::transformToMount(robotPose &in, robotPose &out, bool scale)
{
	matrix pintemp(4, 4), r(3, 3), rtmp1(3, 3);
	matrix pouttemp(4, 4);
	vector<double> vtemp;
	robotPose tmp = in;

	if (scale)
	{
		if (lengthUnits_ == MM)
		{
			//! Convert units from mm
			tmp.x /= 1000.0f;
			tmp.y /= 1000.0f;
			tmp.z /= 1000.0f;
		}
		else if (lengthUnits_ == INCH)
		{
			//! Convert units from inches
			tmp.x /= 39.3701f;
			tmp.y /= 39.3701f;
			tmp.z /= 39.3701f;
		}

		if (angleUnits_ == DEGREE)
		{
			tmp.xrot /= (180.0f / 3.141592654f);
			tmp.yrot /= (180.0f / 3.141592654f);
			tmp.zrot /= (180.0f / 3.141592654f);
		}
	}

	vtemp.push_back(tmp.xrot);
	vtemp.push_back(tmp.yrot);
	vtemp.push_back(tmp.zrot);
	r.rotEulerMatrixConvert(vtemp);
	//! Copy rotation matrix to homogeneous transformation matrix
	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			pintemp.at(x, y) = r.at(x, y);
		}
	}
	pintemp.at(0, 3) = tmp.x;
	pintemp.at(1, 3) = tmp.y;
	pintemp.at(2, 3) = tmp.z;
	pintemp.at(3, 3) = 1.0f;
	pouttemp = *backward_ * pintemp; //JAM pouttemp.matrixMult(*backward_, pintemp, pouttemp); //

	out.x = pouttemp.at(0, 3);
	out.y = pouttemp.at(1, 3);
	out.z = pouttemp.at(2, 3);
	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			r.at(x, y) = pouttemp.at(x, y);
		}
	}

	r.rotMatrixAxisAngleConvert(vtemp);
	out.xrot = vtemp.at(0);
	out.yrot = vtemp.at(1);
	out.zrot = vtemp.at(2);

	return true;
}

LIBRARY_API bool CrpiGazebo::transformFromMount(robotPose &in, robotPose &out, bool scale)
{
	matrix pintemp(4, 4), r(3, 3), rtmp1(3, 3);
	matrix pouttemp(4, 4);
	vector<double> vtemp;

	vtemp.push_back(in.xrot);
	vtemp.push_back(in.yrot);
	vtemp.push_back(in.zrot);
	rtmp1.rotAxisAngleMatrixConvert(vtemp);

	//! Copy rotation matrix to homogeneous transformation matrix
	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			pintemp.at(x, y) = rtmp1.at(x, y);
		}
	}

	pintemp.at(0, 3) = in.x;
	pintemp.at(1, 3) = in.y;
	pintemp.at(2, 3) = in.z;
	pintemp.at(3, 3) = 1.0f;
	pouttemp = (*forward_ * pintemp); //JAM pouttemp.matrixMult(*forward_, pintemp, pouttemp); //
	out.x = pouttemp.at(0, 3);
	out.y = pouttemp.at(1, 3);
	out.z = pouttemp.at(2, 3);
	for (int x = 0; x < 3; ++x)
	{
		for (int y = 0; y < 3; ++y)
		{
			r.at(x, y) = pouttemp.at(x, y);
		}
	}
	r.rotMatrixEulerConvert(vtemp);
	out.xrot = vtemp.at(0);
	out.yrot = vtemp.at(1);
	out.zrot = vtemp.at(2);

	if (scale)
	{
		if (lengthUnits_ == MM)
		{
			//! Convert units to mm
			out.x *= 1000.0f;
			out.y *= 1000.0f;
			out.z *= 1000.0f;
		}
		else if (lengthUnits_ == INCH)
		{
			//! Convert units to inches
			out.x *= 39.3701f;
			out.y *= 39.3701f;
			out.z *= 39.3701f;
		}

		if (angleUnits_ = DEGREE)
		{
			out.xrot *= (180.0f / 3.141592654f);
			out.yrot *= (180.0f / 3.141592654f);
			out.zrot *= (180.0f / 3.141592654f);
		}
	}

	return true;
}

void setModelPose(std::string model, tf::Pose pose)
{
	gazebo_msgs::SetModelState setModelState;
	setModelState.request.model_state.model_name = model; // "sku_fanuctaskfeeder_peg1";
	setModelState.request.model_state.pose = conversion::Convert<tf::Pose, geometry_msgs::Pose>(pose);
	ros::service::call("/gazebo/set_model_state", setModelState);
	// FIXME: add error checking from response. 
	// NOTE: craziness if not clear path to model pose
}
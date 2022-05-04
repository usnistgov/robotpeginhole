///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Robot Interface
//  Workfile:        crpi_universal.h
//  Revision:        1.0 - 24 June, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Universal Robot UR10 interface declarations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GAZEBO_ROBOT
#define GAZEBO_ROBOT
#define LIBRARY_API
//#define TF2_IMPL_CONVERT_H

#include "crpi_util.h"
#include <tf/tf.h>

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

// Boost
#include <boost/format.hpp>
// Error at compile time for non handled convert
#include <boost/static_assert.hpp>
#include <moveit_msgs/RobotTrajectory.h>
#include <urdf/model.h>
//#pragma warning(disable : 4251)
#include <gazebo_msgs/SetJointTrajectory.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetJointProperties.h>
//#include <gazebo/msgs/msgs.hh>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <orocos_kdl/frames_io.hpp>
#include <vector>
#include <sstream>
#include <mutex> // std::mutex
#include <memory>
#include "tiny_kdl.h"

using namespace std;
using namespace Math;
//using namespace Network;

namespace crpi_robot
{
	struct CRobotImpl
	{

		CRobotImpl();
		~CRobotImpl();

		void init(std::string gzGripperTopic);
		static bool isRunning();
		int setGripper(double d);

		// Transformations
		tf::Pose removeGripper(tf::Pose pose);
		tf::Pose addGripper(tf::Pose pose);
		tf::Pose toRobotCoord(tf::Pose pose);
		tf::Pose toWorldCoord(tf::Pose pose);
		tf::Pose retract(tf::Pose pose, tf::Vector3 v);

		int Dwell(double seconds);
		void publishFakeJoints(trajectory_msgs::JointTrajectoryPoint pt);
		tf::Pose FK(std::vector<double> positions);
		std::vector<double> IK(tf::Pose pose);  // querires robot for rcurrent jnt positions.
		std::vector<double> IK(tf::Pose pose, std::vector<double> seed);
		tf::Pose currentPose();
		std::vector<double> currentRobotJoints();
		moveit_msgs::RobotTrajectory calcJointMove(std::vector<double> positions,
												   std::vector<std::string> jointnames,
												   double velscale = 1.0);
		moveit_msgs::RobotTrajectory calcTraj(tf::Pose curpose,
											  tf::Pose rbtFinal,
											  double velscale);
		moveit_msgs::RobotTrajectory calcMoveTo(tf::Pose rbtFinal,
												double velscale = 1.0);
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
		void move(moveit_msgs::RobotTrajectory trajectory);

		void gzMovePt(trajectory_msgs::JointTrajectoryPoint pt);

		void home();
		void readForceTorqueSensor(const geometry_msgs::WrenchStampedConstPtr &_msg);
		void readJointState(const sensor_msgs::JointStateConstPtr  &_msg);

		ros::ServiceClient _gzSetJointTrajSrvClient;	 /**< gazebo ros api plugin service */
		ros::ServiceClient _gzGetJointPropSrvClient;	 /**< gazebo ros api plugin service */
		ros::ServiceClient _gzGetModelStateSrvClient; /**< gazebo ros api plugin service */
		ros::ServiceClient _gzModelStateSrvClient; /**< gazebo ros api plugin service */
		ros::ServiceClient _gzGripperSrvClient;
		ros::ServiceClient _gzRosJointStatesSrvClient;
		ros::ServiceClient _gzRosGuardedMoveSrvClient;

		ros::Publisher _robotJointStatePub; /**< ros subscriber information used for fake joint states */
		ros::Subscriber _forceTorqueSensorSub;
		//ros::Subscriber _gzJointStateSub;  // use OSRF Gazebo joint_state_publisher (latency issues)

		bool bDisplayFT;
		//std::vector<geometry_msgs::Wrench> ft;
		geometry_msgs::Wrench ft;
		sensor_msgs::JointState currentJnts;
		std::string gzGripperTopic;
		std::string _gzrobot_name; /**< gazebo name of robot model defined in sdf */

		std::string joint_state_publisher_topic;
		static ros::NodeHandlePtr nh; /**< node handle for ros app calls */

		// Transforms - robot base<->world, gripper
		bool bDebug;
		bool bFakeJointStatePublisher;
		bool bJntInited;
		bool bJointStatePublisherUpdate;
		tf::Pose basePose;
		tf::Pose basePoseInverse;
		tf::Pose Gripper;
		tf::Pose GripperInv;

		// Kinematics using KDL
		std::shared_ptr<KDL::CTinyKdlSolver> kdl;

		// URDF required
		std::string robotTiplink, robotBaselink, robot_urdf;

		// URDF Derived knowledge
		std::vector<std::string> jointNames;
		std::vector<std::string> linkNames;
		std::vector<double> jointvalues;
		std::vector<double> jointMin;
		std::vector<double> jointMax;
		std::vector<bool> jointHasLimits;
		std::vector<double> jointEffort;
		std::vector<double> jointVelmax;
		std::vector<tf::Vector3> axis;
		std::vector<tf::Vector3> xyzorigin;
		std::vector<tf::Vector3> rpyorigin;
		std::string robotName;

		static ros::AsyncSpinner *_spinner;
		static std::string sRosMasterUrl;
		static std::string sRosPackageName;
		static std::mutex robotmutex;

	};
	struct CWm
	{
	public:
		CWm()
		{
		}

		CWm(std::string name,
			tf::Pose pose);
		~CWm();
		void stop();

		void init(ros::NodeHandlePtr nh, std::shared_ptr<CRobotImpl> robot);
		void start();

		void readConfig();
		std::string getParameter(std::string name, std::string defaultStr = "");
		bool getInstance(std::string name, tf::Pose &pose);
		void storeInstance(std::string name,
						   tf::Pose centroid,
						   tf::Vector3 scale);
		void gzModelStatesCallback(const gazebo_msgs::ModelStates &gzstate_current);
		bool parseURDF();
		std::string _name;
		tf::Pose _location; // location of xyz bottom of object.
		std::vector<CWm> _contains;
		static std::mutex wmmutex;
		ros::Subscriber _gzWorldModel;
		static std::vector<CWm> instances;
		static std::map<std::string, std::string> parameters;
		static bool bReadAllInstances;
		ros::NodeHandlePtr _nh;
		std::string ns;
		std::shared_ptr<CRobotImpl> _robot;
		urdf::Model robot_model;
	};

	struct LIBRARY_API gazeboHandler
	{
		std::mutex mtx;
		//   ulapi_mutex_struct *TCPIPhandle;
		CrpiRobotParams params;
		bool runThread;
		void *rob;
		stringstream moveMe;
		bool poseGood;
		robotPose curPose;
		robotAxes curAxes;
		robotPose curForces;
		robotPose curSpeeds;
		robotIO curIO;
		int curTool;
		double DIO;

		char robotIP[16];
	};

	//! @ingroup Robot
	//!
	//! @brief CRPI interface for the Universal Robot UR10
	//!
	class LIBRARY_API CrpiGazebo
	{
	public:
		std::shared_ptr<CRobotImpl> _arm;
		std::shared_ptr<CWm> _wm;
		bool bDebug;
		//! @brief Default constructor
		//!
		//! @param params Configuration parameters for the CRPI instance of this robot
		//!
		CrpiGazebo(CrpiRobotParams &params);

		//! @brief Default destructor
		//!
		~CrpiGazebo();

		void start();
		void init();

		//! @brief Apply a Cartesian Force/Torque at the TCP, expressed in robot base coordinate system
		//!
		//! @param robotForceTorque are the Cartesian command forces and torques applied at the end-effector
		//!        activeAxes is used to toggle which axes will be slated for active force control. TRUE = ACTIVE, FALSE = INACTIVE
		//!       manipulator is used to toggle which manipulators will be slated for active force control. TRUE = ACTIVE, FALSE = INACTIVE (useful for hands)
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn ApplyCartesianForceTorque(robotPose &robotForceTorque, vector<bool> activeAxes, vector<bool> manipulator);

		//! @brief Apply joint torques
		//!
		//! @param robotJointTorque are the command torques for the respective joint axes
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn ApplyJointTorque(robotAxes &robotJointTorque);

		//! @brief Dock with a specified target object
		//!
		//! @param targetID The name of the object with which the robot should dock
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn Couple(const char *targetID);

		//! @brief Display a message on the operator console
		//!
		//! @param message The plain-text message to be displayed on the operator console
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn Message(const char *message);

		//! @brief Move the robot in a straight line from the current pose to a new pose and stop there
		//!
		//! @param pose        The target 6DOF pose for the robot
		//! @param useBlocking Whether or not to use additional code to ensure blocking on motion commands
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn MoveStraightTo(robotPose &pose, bool useBlocking);

		//! @brief Move the controlled point along a trajectory passing through or near all but the last
		//!        of a series of poses, and then stop at the last pose
		//!
		//! @param poses         An array of 6DOF poses through/near which the robot is expected to pass
		//! @param numPoses      The number of sub-poses in the submitted array
		//! @param accelerations (optional) An array of 6DOF accelaration profiles for each motion
		//!                      associated with the target poses
		//! @param speeds        (optional) An array of 6DOF speed profiles for each motion assiciated
		//!                      with the target poses
		//! @param tolerances    (optional) An array of 6DOF tolerances in length and angle units for the
		//!                      specified target poses
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		//! @note The length of the optional parameter arrays, if provided, must be equal to numPoses.
		//! @note Defining accerlations, speeds, and tolerances does not overwrite the defined default
		//!       values
		//!
		CanonReturn MoveThroughTo(robotPose *poses,
								  int numPoses,
								  robotPose *accelerations = NULL,
								  robotPose *speeds = NULL,
								  robotPose *tolerances = NULL);

		//! @brief Move the controlled pose along any convenient trajectory from the current pose to the
		//!        target pose, and then stop.
		//!
		//! @param pose        The target 6DOF Cartesian pose for the robot's TCP in Cartesian space coordinates
		//! @param useBlocking Whether or not to use additional code to ensure blocking on motion commands
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn MoveTo(robotPose &pose, bool useBlocking);

		//! @brief Get feedback from the robot regarding its current axis configuration
		//!
		//! @param axes Axis array to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotAxes(robotAxes *axes);

		//! @brief Get the measured Cartesian forces from the robot
		//!
		//! @param forces Cartesian force data structure to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotForces(robotPose *forces);

		//! @brief Get I/O feedback from the robot
		//!
		//! @Param io Digital and analog I/O data structure to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotIO(robotIO *io);

		//! @brief Get feedback from the robot regarding its current position in Cartesian space
		//!
		//! @param pose Cartesian pose data structure to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotPose(robotPose *pose);

		//! @brief Get instantaneous Cartesian velocity
		//!
		//! @param speed Cartesian velocities to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotSpeed(robotPose *speed);

		//! @brief Get instantaneous joint speeds
		//!
		//! @param speed Joint velocities array to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotSpeed(robotAxes *speed);

		//! @brief Get joint torques from the robot regarding
		//!
		//! @param torques Axis array to be populated by the method
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn GetRobotTorques(robotAxes *torques);

		//! @brief Move a virtual attractor to a specified coordinate in Cartesian space for force control
		//!
		//! @param pose The 6DOF destination of the virtual attractor
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn MoveAttractor(robotPose &pose);

		//! @brief Move the robot axes to the specified target values
		//!
		//! @param axes        An array of target axis values specified in the current axial unit
		//! @param useBlocking Whether or not to use additional code to ensure blocking on motion commands
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn MoveToAxisTarget(robotAxes &axes, bool useBlocking);

		//! @brief Set the accerlation for the controlled pose to the given value in length units per
		//!        second per second
		//!
		//! @param acceleration The target TCP acceleration
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetAbsoluteAcceleration(double acceleration);

		//! @brief Set the speed for the controlled pose to the given value in length units per second
		//!
		//! @param speed The target Cartesian speed
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetAbsoluteSpeed(double speed);

		//! @brief Set angel units to the unit specified
		//!
		//! @param unitName The name of the angle units in plain text ("degree" or "radian")
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetAngleUnits(const char *unitName);

		//! @brief Set the axis-specific speeds for the motion of axis-space motions
		//!
		//! @param speeds Array of target axial motion speeds
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetAxialSpeeds(double *speeds);

		//! @brief Set specific axial units to the specified values
		//!
		//! @param unitNames Array of axis-specific names of the axis units in plain text
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetAxialUnits(const char **unitNames);

		//! @brief Set the default 6DOF tolerances for the pose of the robot in current length and angle
		//!        units
		//!
		//! @param tolerances Tolerances of the 6DOF end pose during Cartesian motion commands
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetEndPoseTolerance(robotPose &tolerance);

		//! @brief Set the default 6DOF tolerance for smooth motion near intermediate points
		//!
		//! @param tolerances Tolerances of the 6DOF poses during multi-pose motions
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetIntermediatePoseTolerance(robotPose *tolerances);

		//! @brief Set length units to the unit specified
		//!
		//! @param unitName The name of the length units in plain text ("inch," "mm," and "meter")
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetLengthUnits(const char *unitName);

		//! @brief Set a robot-specific parameter (handling of parameter type casting to be handled by the
		//!        robot interface)
		//!
		//! @param paramName The name of the parameter variable to set
		//! @param paramVal  The value to be set to the specified robot parameter
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetParameter(const char *paramName, void *paramVal);

		//! @brief Set the accerlation for the controlled pose to the given percentage of the robot's
		//!        maximum acceleration
		//!
		//! @param percent The percentage of the robot's maximum acceration in the range of [0, 1]
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetRelativeAcceleration(double percent);

		//! @brief Set the digital and analog outputs
		//!
		//! @Param io Digital and analog I/O outputs to set
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetRobotIO(robotIO &io);

		//! @brief Set a specific digital output
		//!
		//! @param dig_out Digital output channel to set
		//! @param val     Value to set the digital output
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetRobotDO(int dig_out, bool val);

		//! @brief Set the attached tool to a defined output rate
		//!
		//! @param percent The desired output rate for the robot's tool as a percentage of maximum output
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetTool(double percent);

		//! @brief Set the speed for the controlled point to the given percentage of the robot's maximum
		//!        speed
		//!
		//! @param percent The percentage of the robot's maximum speed in the range of [0, 1]
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn SetRelativeSpeed(double percent);

		//! @brief Stop the robot's motions based on robot stopping rules
		//!
		//! @param condition The rule by which the robot is expected to stop (Estop category 0, 1, or 2);
		//!                  Estop category 2 is default
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		CanonReturn StopMotion(int condition = 2);

		//! @brief Move the base to a specified position and orientation on a horizontal plane
		//!
		//! @param to Target position in the robot's world frame toward which the robot will attempt to move
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		//! @note This function only uses the x, y, and zrot components of the pose object
		//!
		CanonReturn MoveBase(robotPose &to);

		//! @brief Point the head at an location relative to the robot�s base coordinate frame
		//!
		//! @param to Target pose toward which the head is attempting to point
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		//! @note This function only uses the x, y, and z components of the pose object
		//!
		CanonReturn PointHead(robotPose &to);

		//! @brief Point the appendage at a location relative to the robot�s base coordinate frame
		//!
		//! @param app_ID Identifier of which appendage is being pointed
		//! @param to     Target pose toward which the appendage is attempting to point
		//!
		//! @return SUCCESS if command is accepted and is executed successfully, REJECT if the command is
		//!         not accepted, and FAILURE if the command is accepted but not executed successfully
		//!
		//! @note This function only uses the x, y, and z components of the pose object
		//! @note It is not always possible for the indicated appendage to point exactly along the vector
		//!       specified.  The robot should attempt to get as close as possible.
		//!
		//CanonReturn PointAppendage(CanonRobotAppendage app_ID, robotPose &to);

	private:
		matrix *pin_, *pout_;
		matrix *forward_, *backward_;
		double angleConversion;
		double length_conversion;

		CrpiRobotParams params_;

		void *task;
		unsigned long threadID_;

		bool connectRobot();

		gazeboHandler handle_;

		double maxSpeed_;
		double maxAccel_;

		CanonAngleUnit angleUnits_;
		CanonLengthUnit lengthUnits_;
		CanonAngleUnit axialUnits_[6];

		//! @brief Whether or not we are using socket communications for message passing with the robot
		//!
		bool serialUsed_;
		char COMChannel_[5];
		void *serialID_;
		double curPose_[6];

		bool firstIO_;
		robotIO curIO_;

		//crpi_timer timer_;

		//! @brief Acceleration profile of motions
		//!
		double acceleration_;

		//! @brief Velocity profiles of motions
		//!
		double speed_;

		//! @brief Temporary variable for storing intermediate string values
		//!
		stringstream tempString_;

		//! @brief Message buffer for serial and socket communications
		//!
		char *mssgBuffer_;

		//! @brief Returned data from the robot
		//!
		double *feedback_;

		//! @brief Generate a motion command for the UR robot
		//!
		//! @param moveType  Specify the movement type, either PTP ('P'), LIN ('L'), or force control ('F')
		//! @param posType   Specify the position type, either cartesian ('C') or angular ('A')
		//! @param deltaType Specify the motion delta, either absolute ('A') or relative ('R')
		//! @param input     Vector of 6 position values (note that J3 of the robot is E1, and is thus not
		//!                  used here for angular motion commands)
		//!
		//! @return True if motion string generation was successful, false otherwise
		//!
		bool generateMove(char moveType, char posType, char deltaType, vector<double> &input);

		//! @brief Generate a tool activation request for the UR robot
		//!
		//! @param mode  Specify the mode of actuation of the robot output: binary (B), analog (A), definition (D)
		//! @param value Specify the value of the robot output
		//!
		//! @return True if tool actuation string generation was successful, false otherwise
		//!
		bool generateTool(char mode, double value);

		//! @brief Generate a parameter set request for the UR robot
		//!
		//! @param paramType Specify the parameter to set, see notes for valid parameters
		//! @param subType   Specify the subparameter (if applicable)
		//! @param input     Vector of N values to which the parameters will be set
		//!
		//! @return True if parameter was set successfully, false otherwise
		//!
		//! @note Valid parameters(subparameters) include:
		//!   A(A,R) - Set the acceleration(absolute, relative)
		//!   S(A,R) - Set the speed(absolute, relative)
		//!   F(E,D) - Set force control(enable, disable)
		//!
		bool generateParameter(char paramType, char subtype, vector<double> &input);

		//! @brief Send content of moveMe_ to robot using whatever communication protocol is defined.
		//!
		//bool send ();

		//! @brief Store data from robot in mssgBuffer_ using whatever communication protocol is defined
		//!
		//bool get ();

		bool transformToMount(robotPose &in, robotPose &out, bool scale = true);
		bool transformFromMount(robotPose &in, robotPose &out, bool scale = true);

	}; // CrpiGazebo

} // namespace crpi_robot


#endif

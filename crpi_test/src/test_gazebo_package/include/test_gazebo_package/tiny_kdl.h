#pragma once
#include <orocos_kdl/frames_io.hpp>
#include <orocos_kdl/chain.hpp>
#include <orocos_kdl/frames.hpp>
#include <orocos_kdl/chainiksolverpos_lma.hpp>
#include <orocos_kdl/chainfksolverpos_recursive.hpp>
#include <orocos_kdl/trajectory.hpp>
#include <orocos_kdl/trajectory_segment.hpp>
#include <orocos_kdl/trajectory_composite.hpp>
#include <orocos_kdl/trajectory_stationary.hpp>
#include <orocos_kdl/chainjnttojacsolver.hpp>
#include <orocos_kdl/path_roundedcomposite.hpp>
#include <orocos_kdl/rotational_interpolation.hpp>
#include <orocos_kdl/rotational_interpolation_sa.hpp>
#include <orocos_kdl/velocityprofile_spline.hpp>

// FROM ROS:
#include <kdl_parser/kdl_parser.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <urdf/model.h>
#include "Hacks.h"
#include "conversions.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
namespace KDL
{

using namespace KDL;

// http://wiki.ros.org/pr2_mechanism/Tutorials/Writing%20a%20realtime%20Cartesian%20controller
// http://wiki.ros.org/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20Eigen

struct TrajectoryPoint
{
	std::vector<double> positions;
	std::vector<double> velocities;
	std::vector<double> accelerations;
	double duration;
};

struct CTinyKdlSolver
{
	KDL::Chain chain;
	// forward kinematics solver
	std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
	// velocity-resolved inverse kinematics solver
	std::shared_ptr<KDL::ChainIkSolverPos_LMA> iksolverpos;

	std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	// keeping track of the number of joints we have
	unsigned int dof_;
	bool bDebug;
	//KDL::Jacobian Jtmp; // Jacobian

	CTinyKdlSolver()
	{
		bDebug = false;
	}

	void buildKDLChain(std::string &robot_urdf, std::string root_name, std::string tip_name)
	{
//		Jtmp.resize(chain.getNrOfJoints());
		chain = readChainFromUrdf(robot_urdf, root_name, tip_name);
		std::cout << "KDL chain = \n"
				  << printKdlChain();

		fk_solver_ = std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain));
		iksolverpos = std::shared_ptr<KDL::ChainIkSolverPos_LMA>(new KDL::ChainIkSolverPos_LMA(chain));
		jnt_to_jac_solver = std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain));
	}
	void buildKDLChain(std::vector<Segment> segments)
	{

		for (size_t i = 0; i < segments.size(); i++)
		{
			chain.addSegment(segments[i]);
		}
		std::cout << "KDL chain = \n"
				  << printKdlChain();

		fk_solver_ = std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain));
		iksolverpos = std::shared_ptr<KDL::ChainIkSolverPos_LMA>(new KDL::ChainIkSolverPos_LMA(chain));
		jnt_to_jac_solver = std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain));
	}
	KDL::Chain readChainFromUrdf(std::string &robot_urdf,
								 std::string &root_name, std::string &tip_name)
	{
		KDL::Tree tree;
		KDL::Chain chain;
		urdf::Model robot_model;

		if (!robot_model.initString(robot_urdf))
		{
			ROS_FATAL(" robot_model.initString(robot_urdf) FAILED");
		}

		if (!kdl_parser::treeFromUrdfModel((urdf::ModelInterface &)robot_model, tree))
		{
			cout << "Could not parse robot model into a kdl_tree.\n";
			throw;
		}

		if (!tree.getChain(root_name, tip_name, chain))
		{
			cout << "Could not initialize chain object\n";
			throw;
		}

		return chain;
	}

	std::string printKdlChain()
	{
		std::stringstream ss;
		for (size_t idx = 0; idx < chain.getNrOfSegments(); idx++)
		{
			ss << "* " + chain.getSegment(idx).getJoint().getName();
			ss << " " << chain.getSegment(idx).getJoint().getTypeName();
			ss << " " << chain.getSegment(idx).getJoint().JointAxis();
			ss << " " << chain.getSegment(idx).getJoint().JointOrigin();
			ss << "\n";
		}
		return ss.str();
	}

	tf::Pose FK(std::vector<double> &pos)
	{
		KDL::JntArray q = this->Convert<std::vector<double>, KDL::JntArray>(pos);
		if (bDebug)
		{
			ROS_DEBUG("tf::Pose CTinyKdlSolver::FK\n");
			ROS_DEBUG_STREAM(q(0) << "," << q(1) << "," << q(2) << "," << q(3) << ","
							<< q(4) << "," << q(5)  << "\n");
			assert(q.rows() == dof_);
			assert(fk_solver_);
		}
		KDL::Frame pose;
		if (fk_solver_->JntToCart(q, pose) >= 0)
		{
			if (bDebug)
				std::cout << pose << std::endl;
		}
		else
		{
			ROS_ERROR("fk_solver_->JntToCart(q, pose) FAILED\n");
			__debugbreak();
		}
		return this->Convert<KDL::Frame, tf::Pose>(pose);
	}
	std::vector<double> IK(const std::vector<double> &q_in, const tf::Pose &p)
	{

		//Creation of jntarrays:
		KDL::JntArray q_init = this->Convert<std::vector<double>, KDL::JntArray>(q_in);

		KDL::JntArray q(q_init);

		//Set destination frame
		KDL::Frame F_dest = this->Convert<tf::Pose, KDL::Frame>(p);

		int ret = iksolverpos->CartToJnt(q_init, F_dest, q);
		if (KDL::SolverI::E_NOERROR == ret)
		{
			return this->Convert<KDL::JntArray, std::vector<double>>(q);
		}
		// probably a singularity. Should move some joint to get out of singularity
		return this->Convert<KDL::JntArray, std::vector<double>>(q_init);
	}
	//untested
	std::shared_ptr<Trajectory> GenerateTrajectory(std::vector<tf::Pose> poses)
	{
		//	Path_RoundedComposite(Path_Composite* comp,double radius,double eqradius,RotationalInterpolation* orient, bool aggregate, int nrofpoints);
		Path_RoundedComposite *path = new Path_RoundedComposite(0.2, 0.01, new RotationalInterpolation_SingleAxis());
		for (size_t i = 0; i < poses.size(); i++)
			path->Add(this->Convert<tf::Pose, KDL::Frame>(poses[i]));
		path->Finish();

		VelocityProfile *velpref = new VelocityProfile_Spline();
		//velpref->SetProfileDuration(0, 0, path->PathLength(), 0, double duration)
		velpref->SetProfile(0, path->PathLength());

		// Trajectory defines a motion of the robot along a path.
		std::shared_ptr<Trajectory> traject = std::shared_ptr<Trajectory>(new Trajectory_Segment(path, velpref));

		return traject;
	}
	// Maybe add composite trajectory
	template <typename From, typename To>
	inline To Convert(From f)
	{
		To to;
		//FIXME:?
		//BOOST_STATIC_ASSERT(sizeof(To) == 0);
		assert(0);
		return to;
	}
	template <>
	inline tf::Pose Convert<KDL::Frame, tf::Pose>(KDL::Frame k)
	{
		tf::Pose m;

		m.getOrigin().setX(k.p[0]);
		m.getOrigin().setY(k.p[1]);
		m.getOrigin().setZ(k.p[2]);

		double x, y, z, w;
		k.M.GetQuaternion(x, y, z, w);
		m.setRotation(tf::Quaternion(x, y, z, w));
		return m;
	}
	template <>
	inline KDL::Frame Convert<tf::Pose, KDL::Frame>(tf::Pose m)
	{
		KDL::Frame k;
		k.p[0] = m.getOrigin().x();
		k.p[1] = m.getOrigin().y();
		k.p[2] = m.getOrigin().z();

		k.M = KDL::Rotation::Quaternion(m.getRotation().x(), m.getRotation().y(), m.getRotation().z(), m.getRotation().w());
		return k;
	}
	template <>
	inline KDL::JntArray Convert<std::vector<double>, KDL::JntArray>(std::vector<double> joints)
	{
		KDL::JntArray joint_list(joints.size());

		// Fill in KDL joint list
		for (size_t i = 0; i < joints.size(); i++)
			joint_list(i) = joints[i];
		return joint_list;
	}
	template <>
	inline std::vector<double> Convert<KDL::JntArray, std::vector<double>>(KDL::JntArray joint_list)
	{
		std::vector<double> joints;

		// Fill in KDL joint list
		for (size_t i = 0; i < joint_list.data.size(); i++)
			joints.push_back(joint_list(i));

		return joints;
	}
	static void unitTest(std::string urdf)
	{
		std::string urdfXML =
			"<?xml version=\"1.0\" ?>\n"
			"<!-- =================================================================================== -->\n"
			"<!-- |    This document was autogenerated by xacro from fanuc_only.xacro               | -->\n"
			"<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->\n"
			"<!-- =================================================================================== -->\n"
			"<robot name=\"fanuc_lrmate200id\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n"
			"<link name=\"world\"/>\n"
			"<joint name=\"fanuc_base_link - base\" type=\"fixed\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<parent link=\"world\"/>\n"
			"<child link=\"fanuc_base_link\"/>\n"
			"</joint>\n"
			"<!-- links: main serial chain -->\n"
			"<link name=\"fanuc_base_link\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/base_link.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.278431372549 0.278431372549 0.278431372549 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/base_link.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<mass value=\"0.1\"/>\n"
			"<inertia ixx=\"0.03\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.03\" iyz=\"0.0\" izz=\"0.03\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_1\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_1.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.96 0.76 0.13 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_1.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_2\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_2.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.96 0.76 0.13 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_2.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_3\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_3.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.96 0.76 0.13 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_3.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_4\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_4.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.96 0.76 0.13 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_4.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_5\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_5.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.96 0.76 0.13 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_5.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_link_6\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/link_6.stl\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.15 0.15 0.15 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/link_6.stl\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"0.0226667\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0226667\" iyz=\"0.0\" izz=\"0.0226667\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<joint name=\"fanuc_joint_1\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0.330\"/>\n"
			"<parent link=\"fanuc_base_link\"/>\n"
			"<child link=\"fanuc_link_1\"/>\n"
			"<axis xyz=\"0 0 1\"/>\n"
			"<limit effort=\"0\" lower=\" -2.965\" upper=\"2.965\" velocity=\"7.85\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_joint_2\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0.050 0 0\"/>\n"
			"<parent link=\"fanuc_link_1\"/>\n"
			"<child link=\"fanuc_link_2\"/>\n"
			"<axis xyz=\"0 1 0\"/>\n"
			"<limit effort=\"0\" lower=\" -1.745329\" upper=\"2.530727\" velocity=\"6.63\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_joint_3\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0.330\"/>\n"
			"<parent link=\"fanuc_link_2\"/>\n"
			"<child link=\"fanuc_link_3\"/>\n"
			"<axis xyz=\"0 -1 0\"/>\n"
			"<limit effort=\"0\" lower=\" -2.450966\" upper=\"4.886922\" velocity=\"9.08\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_joint_4\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0.035\"/>\n"
			"<parent link=\"fanuc_link_3\"/>\n"
			"<child link=\"fanuc_link_4\"/>\n"
			"<axis xyz=\" -1 0 0\"/>\n"
			"<limit effort=\"0\" lower=\"-3.315\" upper=\"3.315\" velocity=\"9.6\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_joint_5\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0.335 0 0\"/>\n"
			"<parent link=\"fanuc_link_4\"/>\n"
			"<child link=\"fanuc_link_5\"/>\n"
			"<axis xyz=\"0 -1 0\"/>\n"
			"<limit effort=\"0\" lower=\" -2.18\" upper=\"2.18\" velocity=\"9.51\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_joint_6\" type=\"revolute\">\n"
			"<origin rpy=\"0 0 0\" xyz=\"0.080 0 0\"/>\n"
			"<parent link=\"fanuc_link_5\"/>\n"
			"<child link=\"fanuc_link_6\"/>\n"
			"<axis xyz=\" -1 0 0\"/>\n"
			"<limit effort=\"0\" lower=\" -6.285\" upper=\"6.285\" velocity=\"17.45\"/>\n"
			"</joint>\n"
			"<gazebo reference=\"fanuc_base_link\">\n"
			"<material>Gazebo/Grey</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_1\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_2\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_3\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_4\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_5\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_link_6\">\n"
			"<material>Gazebo/Yellow</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<link name=\"fanuc_finger_1\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/fanuc_finger_1.STL\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.15 0.15 0.15 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/fanuc_finger_1.STL\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"1e-3\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1e-3\" iyz=\"0.0\" izz=\"1e-3\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<link name=\"fanuc_finger_2\">\n"
			"<visual>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/fanuc_finger_2.STL\"/>\n"
			"</geometry>\n"
			"<material name=\"\">\n"
			"<color rgba=\"0.15 0.15 0.15 1.0\"/>\n"
			"</material>\n"
			"</visual>\n"
			"<collision>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<geometry>\n"
			"<mesh filename=\"package://fanuc_lrmate200id_support/meshes/lrmate200id/collision/fanuc_finger_2.STL\"/>\n"
			"</geometry>\n"
			"</collision>\n"
			"<inertial>\n"
			"<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
			"<mass value=\"1\"/>\n"
			"<inertia ixx=\"1e-3\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1e-3\" iyz=\"0.0\" izz=\"1e-3\"/>\n"
			"</inertial>\n"
			"</link>\n"
			"<joint name=\"fanuc_prism1\" type=\"prismatic\">\n"
			"<origin rpy=\" -3.14159  0       1.5708\" xyz=\"0.0 0.02    0.01099\"/>\n"
			"<parent link=\"fanuc_link_6\"/>\n"
			"<child link=\"fanuc_finger_1\"/>\n"
			"<axis xyz=\" -1 0 0 \"/>\n"
			"<limit effort=\"1000\" lower=\" -0.008\" upper=\"0.01\" velocity=\"1000\"/>\n"
			"</joint>\n"
			"<joint name=\"fanuc_prism2\" type=\"prismatic\">\n"
			"<origin rpy=\" -3.14159  0 -1.5708\" xyz=\"0.0 0 -0.01099\"/>\n"
			"<parent link=\"fanuc_link_6\"/>\n"
			"<child link=\"fanuc_finger_2\"/>\n"
			"<axis xyz=\" -1 0 0\"/>\n"
			"<limit effort=\"1000\" lower=\" -0.008\" upper=\"0.0\" velocity=\"1000\"/>\n"
			"</joint>\n"
			"<transmission name=\"trans_fanuc_prism1\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_prism1\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_prism1_motor\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_prism2\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_prism2\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_prism2_motor\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<gazebo reference=\"fanuc_finger_1\">\n"
			"<material>Gazebo/White</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<gazebo reference=\"fanuc_finger_2\">\n"
			"<material>Gazebo/White</material>\n"
			"<turnGravityOff>true</turnGravityOff>\n"
			"</gazebo>\n"
			"<transmission name=\"trans_fanuc_joint_1\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_1\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_1_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_joint_2\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_2\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_2_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_joint_3\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_3\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_3_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_joint_4\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_4\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_4_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_joint_5\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_5\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_5_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_joint_6\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_joint_6\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_joint_6_motor\">\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_prism1\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_prism1\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_prism1_motor\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<transmission name=\"trans_fanuc_prism2\">\n"
			"<type>transmission_interface/SimpleTransmission</type>\n"
			"<joint name=\"fanuc_prism2\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"</joint>\n"
			"<actuator name=\"fanuc_prism2_motor\">\n"
			"<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n"
			"<mechanicalReduction>1</mechanicalReduction>\n"
			"</actuator>\n"
			"</transmission>\n"
			"<gazebo>\n"
			"<static>false</static>\n"
			"<plugin filename=\"libgazebo_ros_control.so\" name=\"gazebo_ros_control\">\n"
			"<robotNamespace>/fanuc_lrmate200id</robotNamespace>\n"
			"<legacynodens>true</legacynodens>\n"
			"</plugin>\n"
			"</gazebo>\n"
			"<gazebo>\n"
			"<plugin filename=\"libgzparallelgripperplugin.so\" name=\"ParallelGripperPlugin\">\n"
			"<robotNamespace>/fanuc_lrmate200id</robotNamespace>\n"
			"<grip_force_close>5</grip_force_close>\n"
			"<joint1>fanuc_lrmate200id::fanuc_prism1</joint1>\n"
			"<joint2>fanuc_lrmate200id::fanuc_prism2</joint2>\n"
			"<grip_kp>10000</grip_kp>\n"
			"<control_topic>/fanuc_lrmate200id/control</control_topic>\n"
			"<debug>1</debug>\n"
			"<fulldebug>0</fulldebug>\n"
			"<collisions>1</collisions>\n"
			"<synchronous> 0  </synchronous>\n"
			"</plugin>\n"
			"</gazebo>\n"
			"</robot>\n";

		//  baselink: fanuc_base_link
		// tiplink: fanuc_link_6

		//if (urdf.empty())
		urdf = urdfXML;
		KDL::CTinyKdlSolver kdl;
		//kdl.buildKDLChain(urdf, "fanuc_base_link", "fanuc_link_6");
		std::vector<Segment> segments =
			{
				Segment(Joint("fanuc_joint_1", KDL::Vector(0,0,0), KDL::Vector(0,0, 1) , Joint::RotAxis), Frame(Vector(0.0, 0.0, 0.330))),
				Segment(Joint("fanuc_joint_2",KDL::Vector(0,0,0), KDL::Vector(0,1, 0) , Joint::RotAxis), Frame(Vector(0.05, 0.0, 0.0))),
				Segment(Joint("fanuc_joint_3",KDL::Vector(0,0,0), KDL::Vector(0,-1, 0) , Joint::RotAxis), Frame(Vector(0.0, 0.0, 0.330))),
				Segment(Joint("fanuc_joint_4",KDL::Vector(0,0,0), KDL::Vector(-1,0, 0) , Joint::RotAxis), Frame(Vector(0.0, 0.0, 0.035))),
				Segment(Joint("fanuc_joint_5",KDL::Vector(0,0,0), KDL::Vector(0,-1, 0) , Joint::RotAxis), Frame(Vector(0.335, 0.0, 0.0))),
				Segment(Joint("fanuc_joint_6",KDL::Vector(0,0,0), KDL::Vector(-1,0,0) , Joint::RotAxis), Frame(Vector(0.08, 0.00, 0.0)))
			};
		kdl.buildKDLChain(segments);
		std::vector<double> home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		std::vector<double> nothome = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
		tf::Pose pose = kdl.FK(home);
		std::cout << "Home = " << conversion::dumpPoseSimple(pose) << "\n";
		std::cout << "Fanuc 200iD Home =  0.47,0.00,0.70,0.000,0.000,0.000,1.000"
				  << "\n";
		std::vector<double> jts = kdl.IK(nothome, pose);
		std::cout << "IK=" << vectorDump<double>(jts) << "\n";
	}
};
}
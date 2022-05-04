/*
 * Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef GAZEBO_ROS_GUARDED_MOVE_PLUGIN_HH
#define GAZEBO_ROS_GUARDED_MOVE_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "gazebo_plugins2/GuardedMove.h"
#include <std_srvs/SetBool.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

// Usage in URDF:
// <gazebo>
    // <plugin name="gazebo_ros_guarded_move"
    //     filename="C:\opt\ros\noetic\gztaskboard\pluginswin\gazebo_ros_guarded_move.dll">
    //     <service_topic>/lrmate/gazebo_ros_guarded_move</service_topic>
    //     <robotNamespace>lrmate</robotNamespace>
    //     <debug> 1 </debug>
    //     <Guard>
    //         <Name>ContactTable</Name>
    //         <SensorID>Ignored</SensorID>
    //         <SubField>Fz</SubField>
    //         <LimitType>DECREASE_BEYOND_LIMIT</LimitType>
    //         <LimitValue>.05</LimitValue>
    //         <RecheckTimeMicroSeconds>20</RecheckTimeMicroSeconds>
    //         <ForceTorqueJoint> fanuc_joint_6</ForceTorqueJoint>
    //     </Guard>
    //     <jointName>fanuc_joint_1, fanuc_joint_2, fanuc_joint_3, fanuc_joint_4, fanuc_joint_5, fanuc_joint_6   </jointName>
    // </plugin>
// </gazebo>

namespace gazebo
{
    class GazeboRosGuardedCmd : public ModelPlugin
    {
    public:
        GazeboRosGuardedCmd();
        ~GazeboRosGuardedCmd();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void updateJointStates();
        bool onServiceCommand(gazebo_plugins2::GuardedMove::Request &_req,
                              gazebo_plugins2::GuardedMove::Response &_res);
        void OnUpdate(const common::UpdateInfo &_info);
        void updateForceTorque();
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr parent_;
        physics::WorldPtr world_;
        physics::ModelPtr model;
        std::vector<physics::JointPtr> joints_;
        physics::Joint_V joints;
        bool bDebug;

        // Single step command (no console input)
        bool bSingleStep;
        std::string singlestep_topic;
        bool OnSingleStepCommand(std_srvs::SetBool::Request &_req,
                                 std_srvs::SetBool::Response &_res);
        ros::ServiceServer singlestep_service;


        // ROS STUFF
        boost::shared_ptr<ros::NodeHandle> _rosnode;
        sensor_msgs::JointState joint_state_;

        ros::ServiceServer guarded_move_service;
        std::string guarded_move_service_topic;

        std::string tf_prefix_;
        std::string robot_namespace_;
        std::vector<std::string> joint_names_;

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

        // Guarded move members
        bool bUpdateWorld;

        std::string guard_Name;
        std::string guard_FTJointName;

        std::string guard_SubField;
        std::string guard_LimitType;
        double guard_LimitValue;
        std::vector<geometry_msgs::Pose> guarded_pose_traj;
        std::vector<trajectory_msgs::JointTrajectoryPoint> guarded_jt_traj;
        size_t guarded_iteration;
        size_t guarded_iteration_max;
        std::vector<std::string> guarded_joint_names;

        // \brief gazebo representation for joint with force torque sensor
        physics::JointPtr ftjoint;

        /// \brief gazebo representation for force/torque  link - not needed?
        physics::LinkPtr ftlink;

         /// \brief gazebo representation for torque  for given joint
        ignition::math::Vector3d curtorque;

        /// \brief gazebo representation for force  for given joint
         ignition::math::Vector3d curforce;

         
        ignition::math::Vector3d linear_velocity;
        ignition::math::Vector3d angular_velocity;

        ignition::math::Vector3d guarded_direction;
        ignition::math::Vector3d guarded_force;
        physics::JointWrench wrench;
   };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosGuardedCmd)
}

#endif //JOINT_STATE_PUBLISHER_PLUGIN_HH

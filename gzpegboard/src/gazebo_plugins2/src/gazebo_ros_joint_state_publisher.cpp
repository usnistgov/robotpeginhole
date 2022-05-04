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
#include <boost/algorithm/string.hpp>
#include <gazebo_plugins2/gazebo_ros_joint_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace gazebo;

#define STRING2(x) #x
#define STRING(x) STRING2(x)

#pragma message ( "Gazebo Version[" STRING(GAZEBO_MAJOR_VERSION) "]")


GazeboRosJointStatePublisher::GazeboRosJointStatePublisher() {}

// Destructor
GazeboRosJointStatePublisher::~GazeboRosJointStatePublisher()
{
    rosnode_->shutdown();
}

void GazeboRosJointStatePublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();
    this->model = _parent;
    this->joints = model->GetJoints();

    // Save model name for fully scoping joint, link and collision names
 #if GAZEBO_MAJOR_VERSION >= 8
   std::string model_name = model->GetName();
    this->robot_namespace_ = parent_->GetName();
#else
     std::string model_name = model->Name();
    this->robot_namespace_ = parent_->Name();
#endif
    if (!_sdf->HasElement("robotNamespace"))
    {
        ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "GazeboRosJointStatePublisher Plugin missing <robotNamespace>, defaults to \"%s\"",
                       this->robot_namespace_.c_str());
    }
    else
    {
        this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        if (this->robot_namespace_.empty())
            this->robot_namespace_ = parent_->GetName();
    }

    bDebug = 0;
    if (_sdf->HasElement("debug"))
        bDebug = _sdf->Get<int>("debug");

    if (!robot_namespace_.empty())
        this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(this->robot_namespace_));

    // std::map<std::string, physics::JointPtr> jnts;
    std::vector<physics::JointPtr> jnts = _parent->GetJoints();
    ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "GazeboRosJointStatePublisher number joints \"%d\"",
                       jnts.size());

    for (size_t i = 0; i < jnts.size(); i++)
    {
        joint_names_.push_back(jnts[i]->GetName());
         ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "GazeboRosJointStatePublisher add joint \"%s\"",
                       jnts[i]->GetName().c_str());
   }

#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world_->SimTime();
#else
    last_update_time_ = this->world_->GetSimTime();
#endif

    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        physics::JointPtr joint = this->parent_->GetJoint(joint_names_[i]);
        if (!joint)
        {
            ROS_FATAL_NAMED("gazebo_ros_joint_state_publisher", "Joint %s does not exist!", joint_names_[i].c_str());
        }
        joints_.push_back(joint);
        ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "GazeboRosJointStatePublisher is going to publish joint: %s", joint_names_[i].c_str());
    }

    service_topic = model_name + "/gazebo_ros_joint_state_publisher_service";
    if (_sdf->HasElement("service_topic"))
        service_topic = _sdf->Get<std::string>("service_topic");

    ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "Starting GazeboRosJointStatePublisher Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName().c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world_->SimTime();
#else
    last_update_time_ = this->world_->GetSimTime();
#endif
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    //this->updateConnection = event::Events::ConnectWorldUpdateBegin (
    //                             boost::bind ( &GazeboRosJointStatePublisher::OnUpdate, this, _1 ) );
    joint_state_publisher_service = rosnode_->advertiseService(service_topic, &GazeboRosJointStatePublisher::onServiceCommand, this);
}

void GazeboRosJointStatePublisher::updateJointStates()
{
    ros::Time current_time = ros::Time::now();
    gzdbg << "GazeboRosJointStatePublisher::updateJointStates "  << std::endl;
 
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize(joints_.size());
    joint_state_.position.resize(joints_.size());
    joint_state_.velocity.resize(joints_.size());

    for (int i = 1; i < joints_.size()+1; i++)
    {
        physics::JointPtr joint = joints_[i-1];
        double velocity = joint->GetVelocity(0);

#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position(0);
#else
        double position = joint->GetAngle(0).Radian();
#endif
        
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = position;
        joint_state_.velocity[i] = velocity;
        gzdbg << "GazeboRosJointStatePublisher::updateJointStates =" << position << std::endl;
   }
    
}

bool GazeboRosJointStatePublisher::onServiceCommand(gazebo_plugins2::GetJointsProperties::Request &_req,
                                               gazebo_plugins2::GetJointsProperties::Response &_res
                                               )
{
    if(bDebug)
        gzdbg << "GazeboRosJointStatePublisher command requested: "  << std::endl;
 
    // get current joint readins
    updateJointStates();

    for (int i = 0; i < joint_state_.name.size(); i++)
    {
        _res.name.push_back(joint_state_.name[i]);
        _res.position.push_back(joint_state_.position[i]);
        _res.velocity.push_back(joint_state_.velocity[i]);
        _res.effort.push_back(0.0);

        if (bDebug)
            ROS_INFO_NAMED("gazebo_ros_joint_state_publisher", "GazeboRosJointStatePublisher Joint %s= %f", joint_state_.name[i].c_str(), joint_state_.position[i]);
    }
    _res.success = true;
    return _res.success;

}
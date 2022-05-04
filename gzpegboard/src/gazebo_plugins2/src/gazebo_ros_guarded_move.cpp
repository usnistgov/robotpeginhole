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
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <gazebo_plugins2/gazebo_ros_guarded_move.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace gazebo;

#define STRING2(x) #x
#define STRING(x) STRING2(x)

#pragma message("Gazebo Version[" STRING(GAZEBO_MAJOR_VERSION) "]")

#include <conio.h>

inline tf::Pose convertgeompose(geometry_msgs::Pose m)
{
    return tf::Pose(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                    tf::Vector3(m.position.x, m.position.y, m.position.z));
}

inline ::ignition::math::Vector3d convertpt(::geometry_msgs::Point pt)
{
    return ::ignition::math::Vector3d(pt.x, pt.y, pt.z);
}
inline ::ignition::math::Pose3d convertpose(::geometry_msgs::Pose pose)
{
    ::ignition::math::Vector3d pt(pose.position.x, pose.position.y, pose.position.z);
    ::ignition::math::Quaterniond q(pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z,
                                    pose.orientation.w);
    return ::ignition::math::Pose3d(pt, q);
}

inline tf::Pose convertpose(::ignition::math::Pose3d pose)
{
    tf::Vector3 pos(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    tf::Quaternion q(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    return tf::Pose(q, pos);
}
/**
     * @brief DumpPoseSimple generate string of xyz origin and rpy rotation from a tf pose.
     * @param pose tf pose
     * @return std::string
     */
inline std::string dumpPoseSimple(tf::Pose pose)
{
    std::stringstream s;
    s << boost::format("%7.3f") % (pose.getOrigin().x()) << "," << boost::format("%7.3f") % (pose.getOrigin().y()) << "," << boost::format("%7.3f") % (pose.getOrigin().z()) << ",";
    s << std::setprecision(3) << boost::format("%5.3f") % pose.getRotation().x() << ","
      << boost::format("%5.3f") % pose.getRotation().y() << ","
      << boost::format("%5.3f") % pose.getRotation().z() << ","
      << boost::format("%5.3f") % pose.getRotation().w();

    return s.str();
}

GazeboRosGuardedCmd::GazeboRosGuardedCmd()
{
}

// Destructor
GazeboRosGuardedCmd::~GazeboRosGuardedCmd()
{
    _rosnode->shutdown();
}

void GazeboRosGuardedCmd::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout << "GazeboRosGuardedCmd: Compiled " << __DATE__ << " " << __TIME__ << "\n"
              << std::flush;

    try
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

#if 0
        sdf::ElementPtr link = _sdf->GetElement("Guard")->GetFirstElement();
        if (link == NULL)
            ROS_FATAL_NAMED("gazebo_ros_guarded_move", "Must have GUARD sdf specified");

        if (!link->HasElement("Name"))
        {
            guard_Name = link->Get<std::string>("Name");
            ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "Guard name \"%s\"", guard_Name.c_str());
        }

        if (!link->HasElement("FTJointName"))
        {
            guard_FTJointName = link->Get<std::string>("FTJointName");
            ftjoint = this->model->GetJoint(guard_FTJointName);
            std::string link1_name = ftjoint->GetJointLink(0)->GetName();
            ftlink = this->model->GetJoint(guard_FTJointName)->GetJointLink(0);
            ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "FTJointName \"%s\"", guard_FTJointName.c_str());
        }
        // if (!link->HasElement("LimitType"))
        // {
        //     guard_LimitType = link->Get<std::string>("LimitType");
        // }
        // if (!link->HasElement("LimitValue"))
        // {
        //     guard_LimitValue = link->Get<double>("LimitValue");
        // }

        std::cout << "Guard name = " << guard_Name << "\n";
        std::cout << "Guard FTJointName = " << guard_FTJointName << "\n";
        //std::cout << "Guard LimitType = " << guard_LimitType << "\n";
        //std::cout << "Guard LimitValue = " << guard_LimitValue << "\n";
#endif

        if (!_sdf->HasElement("robotNamespace"))
        {
            ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "GazeboRosGuardedCmd Plugin missing <robotNamespace>, defaults to \"%s\"",
                           this->robot_namespace_.c_str());
        }
        else
        {
            this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
            if (this->robot_namespace_.empty())
                this->robot_namespace_ = parent_->GetName();
        }
        if (!robot_namespace_.empty())
            this->robot_namespace_ += "/";

        if (!_sdf->HasElement("jointName"))
        {
            ROS_ASSERT("GazeboRosGuardedCmdService Plugin missing jointNames");
        }
        else
        {
            sdf::ElementPtr element = _sdf->GetElement("jointName");
            std::string joint_names = element->Get<std::string>();
            boost::erase_all(joint_names, " ");
            boost::split(joint_names_, joint_names, boost::is_any_of(","));
        }
        bDebug = 1;
        if (_sdf->HasElement("debug"))
            bDebug = _sdf->Get<int>("debug");

        guarded_move_service_topic = model_name + "/gazebo_ros_guarded_move_service";
        if (_sdf->HasElement("service_topic"))
            guarded_move_service_topic = _sdf->Get<std::string>("service_topic");

        // Create ROS node handle
        _rosnode = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(this->robot_namespace_));

        std::vector<physics::JointPtr> jnts = _parent->GetJoints();
        ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "GazeboRosGuardedCmd number joints \"%d\"",
                       jnts.size());

#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = this->world_->SimTime();
#else
        last_update_time_ = this->world_->GetSimTime();
#endif

        // Get the model joints using name list from SDF
        for (unsigned int i = 0; i < joint_names_.size(); i++)
        {
            physics::JointPtr joint = this->parent_->GetJoint(joint_names_[i]);
            if (!joint)
            {
                ROS_FATAL_NAMED("gazebo_ros_guarded_move_service", "Joint %s does not exist!", joint_names_[i].c_str());
            }
            joints_.push_back(joint);
            ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "GazeboRosGuardedCmd is going to publish joint: %s", joint_names_[i].c_str());
        }

        if (_sdf->HasElement("forceTorqueJoint"))
        {
            std::string ftJointName = _sdf->Get<std::string>("forceTorqueJoint");
            ftjoint = this->parent_->GetJoint(ftJointName);
        }
        if (ftjoint == NULL)
        {
            ROS_ASSERT("GazeboRosGuardedCmdService forceTorqueJoint Missing or Invalid!");
        }

        this->update_rate_ = 100.0;
        if (!_sdf->HasElement("updateRate"))
        {
            ROS_WARN_NAMED("gazebo_ros_guarded_move_service", "GazeboRosJointStatePublisher Plugin (ns = %s) missing <updateRate>, defaults to %f",
                           this->robot_namespace_.c_str(), this->update_rate_);
        }
        else
        {
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
        }

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0)
        {
            this->update_period_ = 1.0 / this->update_rate_;
        }
        else
        {
            this->update_period_ = 0.0;
        }
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = this->world_->SimTime();
#else
        last_update_time_ = this->world_->GetSimTime();
#endif

        tf_prefix_ = tf::getPrefixParam(*_rosnode);

#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = this->world_->SimTime();
#else
        last_update_time_ = this->world_->GetSimTime();
#endif

        linear_velocity = ignition::math::Vector3d(0, 0, 0);
        angular_velocity = ignition::math::Vector3d(0, 0, 0);

// Listen to the update event. This event is broadcast every
// simulation iteration.
#if 1
        ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "Starting GazeboRosGuardedCmd Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName().c_str());
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboRosGuardedCmd::OnUpdate, this, _1));
        bUpdateWorld = false;

        if (singlestep_topic.empty())
            singlestep_topic = "GazeboRosGuardedCmdDebug";
        singlestep_service = _rosnode->advertiseService(singlestep_topic, &GazeboRosGuardedCmd::OnSingleStepCommand, this);

        guarded_move_service = _rosnode->advertiseService(guarded_move_service_topic, &GazeboRosGuardedCmd::onServiceCommand, this);
#endif
    }
    catch (...)
    {
        ROS_FATAL_NAMED("gazebo_ros_guarded_move_service", "Exception thrown on LOAD plugin");
    }
}

void GazeboRosGuardedCmd::updateJointStates()
{
    try
    {
        ros::Time current_time = ros::Time::now();
        if (bDebug)
            std::cout << "GazeboRosGuardedCmd::updateJointStates " << std::endl;

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(joints_.size());
        joint_state_.position.resize(joints_.size());
        joint_state_.velocity.resize(joints_.size());

        for (int i = 0; i < joint_names_.size(); i++)
        {
            physics::JointPtr joint = joints_[i];
            double velocity = joint->GetVelocity(0);

#if GAZEBO_MAJOR_VERSION >= 8
            double position = joint->Position(0);
#else
            double position = joint->GetAngle(0).Radian();
#endif

            joint_state_.name[i] = joint_names_[i];
            joint_state_.position[i] = position;
            joint_state_.velocity[i] = velocity;
            if (bDebug)
                std::cout << "GazeboRosGuardedCmd::updateJointStates =" << position << std::endl;
        }
    }
    catch (...)
    {
        std::cerr << "GazeboRosGuardedCmd::updateJointStates() Exception\n";
    }
}

bool GazeboRosGuardedCmd::onServiceCommand(gazebo_plugins2::GuardedMove::Request &_req,
                                           gazebo_plugins2::GuardedMove::Response &_res)
{
    try
    {
        if (bDebug)
            std::cout << "GazeboRosGuardedCmd command requested: " << std::endl;
        guarded_direction = convertpt(_req.direction); // ::geometry_msgs::Point
        guarded_force = convertpt(_req.maxforce);      // ::geometry_msgs::Point
        guarded_pose_traj = _req.pose_traj;
        guarded_jt_traj = _req.jt_traj;
        guarded_iteration = 0;
        guarded_iteration_max = _req.jt_traj.size();
        guarded_joint_names=_req.joint_names;
        if(guarded_joint_names.size() != _req.jt_traj[0].positions.size())
        {
            ROS_FATAL_NAMED("gazebo_ros_guarded_move", "Mismatched joint names and positions size");
        }

        bUpdateWorld = true;
        if (bDebug)
        {
            std::cout << "GazeboRosGuardedCmd guarded_direction=" << guarded_direction.X() << " "
                      << guarded_direction.Y() << " " << guarded_direction.Z() << "\n";
            std::cout << "GazeboRosGuardedCmd guarded_force=" << guarded_force.X() << " "
                      << guarded_force.Y() << " " << guarded_force.Z() << "\n";

            std::cout << "GazeboRosGuardedCmd guarded_jnt_traj=\n";
            for (size_t i = 0; i < guarded_jt_traj.size(); i++)
            {
                for (size_t j = 0; j < guarded_jt_traj[i].positions.size(); j++)
                    std::cout << guarded_jt_traj[i].positions[j] << ",";
                std::cout << "\n";
            }
        }

#if 0
        // get current joint readins
        //updateJointStates();

        for (int i = 0; i < joint_names_.size(); i++)
        {
            _res.name.push_back(joint_state_.name[i]);
            _res.position.push_back(joint_state_.position[i]);
            _res.velocity.push_back(joint_state_.velocity[i]);
            _res.effort.push_back(0.0);

            if (bDebug)
                ROS_INFO_NAMED("gazebo_ros_guarded_move_service", "GazeboRosGuardedCmd Joint %s= %f", joint_state_.name[i].c_str(), joint_state_.position[i]);
        }
#endif
        _res.success = true;
    }
    catch (...)
    {
        std::cerr << "GazeboRosGuardedCmd::onServiceCommand() Exception\n";
        _res.success = false;
    }
    return _res.success;
}

void GazeboRosGuardedCmd::updateForceTorque()
{
    // if (bDebug)
    //     std::cout << "GazeboRosGuardedCmd::updateForceTorque()" << std::endl;
    wrench = this->ftjoint->GetForceTorque(0);
#if GAZEBO_MAJOR_VERSION >= 8
    curforce = wrench.body2Force;
    curtorque = wrench.body2Torque;
#else
    curforce = wrench.body2Force.Ign();
    curtorque = wrench.body2Torque.Ign();
#endif
    if (this->bDebug)
        std::cout << "Force =" << curforce.X() << ","
                  << curforce.Y() << ","
                  << curforce.Z() << std::endl;
}
bool GazeboRosGuardedCmd::OnSingleStepCommand(std_srvs::SetBool::Request &_req,
                                              std_srvs::SetBool::Response &_res)
{
    // single to single step a service
    bSingleStep=true;
    return true;
}

void GazeboRosGuardedCmd::OnUpdate(const common::UpdateInfo &_info)
{
    try
    {
        if (!bUpdateWorld)
            return;

        // Sginle step via rosservice call - not tested
        // if (bSingleStep)
        // {
        //     std::cout << "GazeboRosGuardedCmd::OnUpdate Singlestep\n";
        //     bSingleStep=0;
        // }
        // else
        //     return;



        if (bDebug)
            std::cout << "GazeboRosGuardedCmd::OnUpdate()" << std::endl;

        std::cout << "Before "; updateForceTorque();

#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = this->world_->SimTime();
#else
        common::Time current_time = this->world_->GetSimTime();
#endif
        if (current_time < last_update_time_)
        {
            ROS_WARN_NAMED("gazebo_ros_guarded_move_service", "Negative joint state update time difference detected.");
            last_update_time_ = current_time;
        }

        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_)
        {

            // Synchronous get characgter from ROS console - will pause here waiting
            int c = _getch(); // call your non-blocking input function

#if 1

            // Negative forces?
            if (curforce.Z() > this->guarded_force.Z())
            {
                std::cout << "Turn off guarded move\n";
                this->bUpdateWorld = false;
                return;
            }

            ignition::math::Pose3d _pose = model->WorldPose();

            guarded_iteration++;
            if (guarded_iteration >= guarded_iteration_max)
            {
                std::cout << "Turn off guarded move -  exhaust all iterative moves\n";
                this->bUpdateWorld = false;
                return;
            }

            if (bDebug)
            {
                tf::Pose tfpose = convertgeompose(guarded_pose_traj[guarded_iteration]);
                std::cout << "GazeboRosGuardedCmd::OnUpdate pose=" << dumpPoseSimple(tfpose) << "\n";
            }

            for (unsigned int i = 0; i < guarded_jt_traj[guarded_iteration].positions.size(); ++i)
            {
                if(i>=guarded_joint_names.size())
                {
                    std::cout << "ERROR: GazeboRosGuardedCmd::OnUpdate mismatch joint name and joint position sizes\n";
                }
                std::string joint_name = guarded_joint_names[i];
                physics::JointPtr joint = model->GetJoint(joint_name);

                if (joint)
                {
#if GAZEBO_MAJOR_VERSION >= 9
                    joint->SetPosition(0, guarded_jt_traj[guarded_iteration].positions[i], true);
#else
                    joint->SetPosition(0, guarded_jt_traj[guarded_iteration].positions[i]);
#endif
                }
                else
                {
                    std::cout << "ERROR: GazeboRosGuardedCmd::OnUpdate  joint name not found "  << joint_name <<"\n";
                }
            }
            // iS this necessary? MUST BE WORLD COORDINATES
            //this->model->SetWorldPose(convertpose(guarded_pose_traj[guarded_iteration]));
            std::cout << "After "; updateForceTorque();

#endif
            last_update_time_ += common::Time(update_period_);
        }
    }
    catch (...)
    {
        std::cerr << "GazeboRosGuardedCmd::OnUpdate() Exception\n";
    }
}
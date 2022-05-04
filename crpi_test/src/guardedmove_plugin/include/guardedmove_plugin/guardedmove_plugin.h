#ifndef GZGUARDEDMOVEPLUGIN_H
#define GZGUARDEDMOVEPLUGIN_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <std_srvs/SetBool.h>


#include <string>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// References:
// https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin

namespace gazebo
{

/**
 * @brief The gzParallelGripperPlugin class is based on code
 * from https://github.com/Pavel-P/test_gripper which
 * uses force control to do simulated grasping.
 *
 * gzParallelGripperPlugin works for parallel grippers (two finger grippers)
 * and accepts a clost or open command from the custom protobuf command
 * GripCommand.
 *
\code

\endcode
where an enable =true closes the gripper and an enable-false
opens the gripper. The algorithm uses force control to close
on the grasping object, and upon contact by a finger a virtual joint
is created between the finger and the object. When both fingers
have created a virtual joint to the object, the grasping close
is completed and the state becomes equal to one.
 */
class  GuardedMovePlugin : public ModelPlugin
{

    /**
     * @brief The VirtualJoint class creates a virtual joint between a grasped object and the gripper finger links.
     * Useful since most gripped objects cannot be grasped so this provides an efficient kludge to allow grasping.
     */
    class VirtualJoint
        {
          public: physics::JointPtr joint;
          public: GuardedMovePlugin* parent;
          public: physics::CollisionPtr collision1;
          public: physics::CollisionPtr collision2;
          public: ignition::math::Vector3d contactForce;

          public: static bool CheckValidity(VirtualJoint* v);

          public: VirtualJoint(GuardedMovePlugin* p,
                               physics::JointPtr j,
                               physics::CollisionPtr c1,
                               physics::CollisionPtr c2,
                               ignition::math::Vector3d f);
          public: ~VirtualJoint() {}
        };
  public:
    /**
     * @brief gzParallelGripperPlugin constructor to create the plugin.
     */
    GuardedMovePlugin ();
    ~GuardedMovePlugin ();

    /**
     * @brief Loads the gripper model.
     * @param _parent model parent
     * @param _sdf  XML describing element options for setup in plugin.
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief onUpdate Called by the gazebo simulation world update event
     * @param _info
     */
    void onUpdate (const common::UpdateInfo & _info);

    /**
     * @brief publishGripperState publishes state of gripper  grasping.
     */
    //void publishGripperState();

    /**
     * @brief OnGripperCommand Handle incoming message
     * @param[in] msg enable (close) or disable (open) gripper
     */
    bool OnGripperCommand(std_srvs::SetBool::Request &_req,
               std_srvs::SetBool::Response &_res);


    /**
     * @brief Detach detach grasped object from fingers by deleted virtual joint(s) between object and
     * gripper finger links.
     */
    void Detach();

    /**
     * @brief CheckAttach checks collision between parallel gripper finger link collisions and any object.
     * If found will create a virtual joint between collision object and collision links.
     * @return number of virtual joints. Zero if no contact.
     */
    int CheckAttach();

    /**
     * @brief SetLinksGravityFlag Set whether gravity affects a link.
     * @param objmodel  object to turn gravity on/off
     * @param bFlag  true is on, false is off
     */
    void SetLinksGravityFlag(physics::ModelPtr objmodel, bool bFlag);

private:
    /// \brief forces for gripper finger 1 and finger 2
    double force1, force2;

    /// \brief gazebo representation for gripper finger1 joint
    physics::JointPtr joint1;

    /// \brief gazebo representation for gripper finger2 joint
    physics::JointPtr joint2;

    /// \brief gazebo representation for gripper finger1 link
    physics::LinkPtr link1;

    /// \brief gazebo representation for gripper finger2 link
    physics::LinkPtr link2;

    /// \brief gripper force used for closing. Input from sdf. Default to 5N.
    double grip_force_close;

    /// \brief gripper force used for opeing. Input from sdf. Default to 2.5N.
    double grip_force_open;

    /// \brief gripper p in gripper force pid.
    double grip_kp;

    /// \brief sdf plugin debug flag. If true, debug message sent to console
    int bDebug;

    /// \brief sdf plugin fully debug flag. If true, many many debug message sent to console
    int bFullDebug;

    /// \brief sdf plugin collision flag. If true, collisions between gripper fingers and object handled.
    int bCollisions;

    /// \brief message cmd grasping flag. true if gripper to close and  false if to open gripper.
    bool grip_enabled;

    /// \brief message status grasping flag. true if gripper closed and grasping false if open and not grasping.
    int bGrasping;

    /// \brief synchronous service communication flag - if true, wait until gripper has completed actino before returnsing.
    int bSynchronous;

    /// \brief name to use for accessing robot
    std::string robot_namespace = "";


//    /// \brief A node used for gazebo transport
//    gazebo::transport::NodePtr node;

    /// \brief nano seconds between two updates
    common::Time updateRate;

    /// \brief time of last update
    common::Time last_update_time_;
    double update_period_;

    /// \brief fully decorated gripper joints  name
    std::vector<std::string> joint_names;

    /// \brief gazebo physics joint pointer for each gripper finger joint
    std::vector<physics::JointPtr> gzjoints;

    /// \brief gazebo physics link pointer for each gripper finger link
    std::vector<physics::LinkPtr> gzlinks;

    /// \brief vector of virtual joints that attach object to gripper finger joints.
    std::vector<VirtualJoint*> virtualJoints;

    /**

      */
    std::unique_ptr<ros::NodeHandle> rosnode;
    ros::ServiceServer grip_service;
    std::string control_topic;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model
    physics::ModelPtr model;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief virtual joint naming counter.
    unsigned int jointId;

    ignition::math::Vector3d linear_velocity;
    ignition::math::Vector3d angular_velocity;

    ////////////////////////////////////////////////////////
    // Contact manager and filtering
    ////////////////////////////////////////////////////////

    /// \brief gazebo world contact manager.
    physics::ContactManager* contactManager;

     /// \brief gripper finger collision names
    std::vector<std::string> collisionNames;

    /// \brief list of contacts
    std::vector< ::gazebo::msgs::Contact> contacts;

    /// \brief list of gripper finger link collision objects (unused).
    std::map<std::string, physics::CollisionPtr> collisionElems;



    /// \brief contact filter subscriber topic name
    std::string contacttopic;

    /// \brief A node used for gazebo transport to contacts
    gazebo::transport::NodePtr node;

    /// \brief subscriber to filtered contact update
    transport::SubscriberPtr contactSub;

    /**
     * Gets called upon detection of contacts.
     * A list of contacts is passed in \_msg. One contact has two bodies, and only
     * the ones where one of the bodies is a gripper link are considered.
     * Each contact consists of a *list* of forces with their own origin/position each
     * (e.g. when the object and gripper are colliding at several places).
     * The averages of each contact's force vectors along with their origins is computed.
     * This "average contact force/origin" for each contact is then added to the \e this->contacts map.
     * If an entry for this object/link pair already exists, the average force (and its origin)
     * is *added* to the existing force/origin, and the average count is increased. This is to get
     * the average force application over time between link and object.
     */
    void OnContact(const ConstContactsPtr& msg);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GuardedMovePlugin)

}
#endif // GZPARALLELGRIPPERPLUGIN_H
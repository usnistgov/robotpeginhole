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

#ifndef CONVERSIONS
#define CONVERSIONS

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
//#include <ros/ros.h>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
//#include <orocos_kdl/frames_io.hpp>

#include <tf/tf.h>
#include <urdf/model.h>

// Boost
#include <boost/format.hpp>
// Error at compile time for non handled convert
#include <boost/static_assert.hpp>

#pragma warning(disable : 4251)

#include <vector>
#include <sstream>
#include <mutex> // std::mutex
#include <memory>

namespace conversion
{
    /*!
     * \brief Empty conversion of type from into type to. If called, asserts.
     * \param f is defined in the template corresponding to the "From" typename.
     * \return to is defined in the template corresponding "To"  typename
     */
    template <typename From, typename To>
    inline To Convert(From f)
    {
        To to;
        //FIXME:?
        //BOOST_STATIC_ASSERT(sizeof(To) == 0);
        assert(0);
        return to;
    }

    /*!
     * \brief Convert geometry_msgs::Pose into tf::Pose.
     * \param pose is copy constructor of geometry_msgs::Pose.
     * \return tf::Pose
     */
    template <>
    inline tf::Pose Convert<geometry_msgs::Pose, tf::Pose>(geometry_msgs::Pose m)
    {
        return tf::Pose(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                        tf::Vector3(m.position.x, m.position.y, m.position.z));
    }
    /**
     * \brief Convert std::vector<double> into tf::Pose.
     * \param ds are an array of 6 or 7 doubles to define tf Pose.
     * If 6 double use rpy if 7 use quaternion.
     * \return tf::Pose
     */
    template <>
    inline tf::Pose Convert<std::vector<double>, tf::Pose>(std::vector<double> ds)
    {
        tf::Pose pose;
        if (ds.size() == 6)
        {
            std::array<double, 6> arr;
            std::copy_n(ds.begin(), 6, arr.begin());
            pose = Convert<std::array<double, 6>, tf::Pose>(arr);
        }
        else if (ds.size() == 7)
        {
            std::array<double, 7> arr;
            std::copy_n(ds.begin(), 7, arr.begin());
            pose = Convert<std::array<double, 7>, tf::Pose>(arr);
            // message should now contain object - can't really detect if exists
        }
        else
        {
            assert(0);
        }
        return pose;
    }
    /*!
     * \brief Convert tf::Pose pose into an  geometry_msgs::Pose pose.
     * \param m is a tf::Pose transform matrix.
     * \return  geometry_msgs::Pose pose
     */
    template <>
    inline geometry_msgs::Pose Convert<tf::Pose, geometry_msgs::Pose>(tf::Pose m)
    {
        geometry_msgs::Pose p;
        p.position.x = m.getOrigin().x();
        p.position.y = m.getOrigin().y();
        p.position.z = m.getOrigin().z();
        p.orientation.x = m.getRotation().x();
        p.orientation.y = m.getRotation().y();
        p.orientation.z = m.getRotation().z();
        p.orientation.w = m.getRotation().w();
        return p;
    }

    template <>
    inline tf::Vector3 Convert<urdf::Vector3, tf::Vector3>(urdf::Vector3 v)
    {
        return tf::Vector3(v.x, v.y, v.z);
    }

    inline std::vector<double> getTranslation(tf::Pose p)
    {
        std::vector<double> trans = {p.getOrigin().getX(), p.getOrigin().getY(), p.getOrigin().getZ()};
        return trans;
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
        double roll = 0, pitch = 0, yaw = 0;
        tf::Matrix3x3 m;
        m.setRotation(pose.getRotation());
        m.getRPY(roll, pitch, yaw);

        s << std::setprecision(3) << boost::format("%5.3f") % pose.getRotation().x() << ","
          << boost::format("%5.3f") % pose.getRotation().y() << ","
          << boost::format("%5.3f") % pose.getRotation().z() << ","
          << boost::format("%5.3f") % pose.getRotation().w();

        return s.str();
    }
    inline std::string dumpPoseRPY(tf::Pose pose)
    {
        std::stringstream s;
        s << boost::format("%7.3f") % (pose.getOrigin().x()) << "," << boost::format("%7.3f") % (pose.getOrigin().y()) << "," << boost::format("%7.3f") % (pose.getOrigin().z()) << ",";
        double roll = 0, pitch = 0, yaw = 0;
        tf::Matrix3x3 m;
        m.setRotation(pose.getRotation());
        m.getRPY(roll, pitch, yaw);

        s << std::setprecision(3) << boost::format("%5.3f") % roll << ","
          << boost::format("%5.3f") % pitch << ","
          << boost::format("%5.3f") % yaw;

        return s.str();
    }
    /**
     * @brief dumpVectorSimple  string of xyz origin from a tf vector.
     * @param v tf vector
     * @return std::string
     */
    inline std::string dumpVectorSimple(tf::Vector3 v)
    {
        std::stringstream s;
        s << std::setprecision(3) << boost::format("%7.2f") % (v.x()) << "," << boost::format("%7.2f") % (v.y()) << "," << boost::format("%7.2f") % (v.z()) << ",";
        return s.str();
    }

    // template <>
    // inline tf::Pose Convert<KDL::Frame, tf::Pose>(KDL::Frame k)
    // {
    //     tf::Pose m;

    //     m.getOrigin().setX(k.p[0]);
    //     m.getOrigin().setY(k.p[1]);
    //     m.getOrigin().setZ(k.p[2]);

    //     double x, y, z, w;
    //     k.M.GetQuaternion(x, y, z, w);
    //     m.setRotation(tf::Quaternion(x, y, z, w));
    //     return m;
    // }
    // template <>
    // inline KDL::Frame Convert<tf::Pose, KDL::Frame>(tf::Pose m)
    // {
    //     KDL::Frame k;
    //     k.p[0] = m.getOrigin().x();
    //     k.p[1] = m.getOrigin().y();
    //     k.p[2] = m.getOrigin().z();

    //     k.M = KDL::Rotation::Quaternion(m.getRotation().x(), m.getRotation().y(), m.getRotation().z(), m.getRotation().w());
    //     return k;
    // }
    // template <>
    // inline KDL::JntArray Convert<std::vector<double>, KDL::JntArray>(std::vector<double> joints)
    // {
    //     KDL::JntArray joint_list(joints.size());

    //     // Fill in KDL joint list
    //     for (size_t i = 0; i < joints.size(); i++)
    //         joint_list(i) = joints[i];
    //     return joint_list;
    // }
    // template <>
    // inline std::vector<double> Convert<KDL::JntArray, std::vector<double>>(KDL::JntArray joint_list)
    // {
    //     std::vector<double> joints;

    //     // Fill in KDL joint list
    //     for (size_t i = 0; i < joint_list.data.size(); i++)
    //         joints.push_back(joint_list(i));

    //     return joints;
    // }

}

#endif

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
#include "crpi_util.h"

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
#include <orocos_kdl/frames_io.hpp>

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
    inline int RotMatrix2Rpy(tf::Matrix3x3 m, double &r, double &p, double &y)
    {
        p = atan2(-m[0][2], sqrt(SQ(m[0][0]) + SQ(m[0][1])));

        if (fabs(p - M_PI_2) < RPY_P_FUZZ)
        {
            r = atan2(m[1][0], m[1][1]);
            p = M_PI_2;
            /* force it */
            y = 0.0;
        }
        else if (fabs(p + M_PI_2) < RPY_P_FUZZ)
        {
            r = -atan2(m[1][2], m[1][1]);
            p = -M_PI_2;
            /* force it */
            y = 0.0;
        }
        else
        {
            r = atan2(m[1][2], m[2][2]);
            y = atan2(m[0][1], m[0][0]);
        }
        return 0;
    }

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

    template <>
    inline tf::Pose Convert<Math::matrix, tf::Pose>(Math::matrix forward)
    {
        tf::Pose p;
        tf::Vector3 trans((forward).at(0, 3), (forward).at(1, 3), (forward).at(2, 3));
        p.setOrigin(trans);
        std::vector<double> q;
        (forward).rotQuaternionMatrixConvert(q);
        p.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));
        return p;
    }
    template <>
    inline robotPose Convert<tf::Pose, robotPose>(tf::Pose p)
    {
        robotPose pose;
        pose.x = p.getOrigin().getX();
        pose.y = p.getOrigin().getY();
        pose.z = p.getOrigin().getZ();

        //RotMatrix2Rpy(p.getBasis(), pose.xrot, pose.yrot, pose.zrot);

        tf::Matrix3x3 m(p.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose.xrot = roll;
        pose.yrot = pitch;
        pose.zrot = yaw;

        return pose;
    }

    inline Math::matrix getRotation(tf::Pose p)
    {
        Math::matrix r(3, 3);
        tf::Matrix3x3 m = p.getBasis();
        r.at(0, 0) = m[0][0];
        r.at(0, 1) = m[0][1];
        r.at(0, 2) = m[0][2];

        r.at(1, 0) = m[1][0];
        r.at(1, 1) = m[1][1];
        r.at(1, 2) = m[1][2];

        r.at(2, 0) = m[2][0];
        r.at(2, 1) = m[2][1];
        r.at(2, 2) = m[2][2];

        return r;
    }

    template <>
    inline tf::Pose Convert<robotPose, tf::Pose>(robotPose pose)
    {
        tf::Pose p;
        p.setOrigin(tf::Vector3(pose.x, pose.y, pose.z));
        p.setRotation(tf::Quaternion(pose.yrot, pose.zrot, pose.xrot));

        // Math::matrix r(3, 3);
        // std::vector<double> rpy = { pose.xrot, pose.yrot, pose.zrot};
        // r.rotEulerMatrixConvert(rpy);
        // std::vector<double> q;
        // r.rotMatrixQuaternionConvert(q);
        // p.setRotation(tf::Quaternion( q[1], q[2], q[3],q[0]));
        return p;
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
        RotMatrix2Rpy(m, roll, pitch, yaw);

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
        m.getRPY( roll, pitch, yaw);

        s << std::setprecision(3) << boost::format("%5.3f") % (roll * 180.0 / M_PI) << ","
          << boost::format("%5.3f") % (pitch * 180.0 / M_PI) << ","
          << boost::format("%5.3f") % (yaw * 180.0 / M_PI) ;

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

}

#endif

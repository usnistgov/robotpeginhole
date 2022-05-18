/*!
* \file c:\opt\ros\noetic\crpi_test\src\test_gazebo_package\include\test_gazebo_package\wm.h
* \author John Michaloski <john.michaloski@hotmail.com>
* \version 0.1
* \date 01/05/2022
* \brief 
* \remarks None
*/


#ifndef GAZEBOROBOTWM
#define GAZEBOROBOTWM

/*! Importation of librairies*/
#include <vector>
#include <tf/tf.h>
#include <boost/algorithm/string.hpp>
#include "traj_math.h"
#include "crpi_util.h"
#include "crpi_gazebo.h"
#include "tiny_kdl.h"


/**
 * @brief  scaling used from CAD to Gazeboe STL model
 * should be constant. Tricky changing on the fly.
 */
extern double gzPegboardScaleFactor;  


struct CPegArrayHole
{
    std::string name;
    std::string state;
    std::string holetype;
    tf::Vector3 offset;
    tf::Vector3  wCurlocation;
    CPegArrayHole()
    {
        name = "empty";
    }
    CPegArrayHole(std::string name, std::string state, std::string holetype, tf::Vector3 offset)
    {
        this->name = name;
        this->state = state;
        this->holetype = holetype;
        this->offset = offset;
    }
    tf::Vector3 topOfHole(tf::Vector3 centroid)
    {
        return centroid + offset;
    }
    tf::Vector3 curTopZLocation()
    {
        // location is also an offset
        tf::Matrix3x3 m(centroid.getRotation());
        tf::Vector3 rotOffset = m * offset ; // rotate pegarray offset matrix if necessary
        tf::Vector3 center = centroid.getOrigin();
        std::cout << name << " centroid" << conversion::dumpPoseRPY(centroid) << "\n";
        std::cout << name << " location" << conversion::dumpVectorSimple(offset) << "\n";
        std::cout << name << " offset" << conversion::dumpVectorSimple(rotOffset) << "\n";
        std::cout << name << " center" << conversion::dumpVectorSimple(center) << "\n";

        std::cout << name << " curTopZLocation" << conversion::dumpVectorSimple(tf::Vector3(center.x() + rotOffset.x(), 
        center.y() + rotOffset.y(), 
        center.z() + rotOffset.z() + zlen())) << "\n";

        return tf::Vector3(center.x() + rotOffset.x(),
                           center.y() + rotOffset.y(),
                           center.z() + rotOffset.z() + zlen());
    }
    double xlen() { return _xlen*_scale; }
    double ylen() { return _ylen*_scale; }
    double zlen() { return _zlen*_scale; }
    double &scale() { return _scale; }
    /////////////////////////////////////////////////////////////
    double _xlen;     // constant x length
    double _ylen;     // constant x length
    double _zlen;     // constant x length
    double _scale;    // constant x length
    tf::Pose centroid;
};

struct CPegHoleArray : public std::vector<CPegArrayHole>
{
    CPegHoleArray()
    {
    }
    CPegHoleArray(std::string name, const std::vector<CPegArrayHole> &other)
    {
        copy(other.begin(), other.end(), back_inserter(*this));
        _name=name;
        _scale = 0.002;
        _xlen = 44.45000076293945/10.0 * 2. ;
        _ylen = 3.174999237060547/10.0 * 2. ;
        _zlen =  8.8; // 88.90000915527344 ; // empiral observation using move in Gazebo
        for(size_t i=0; i< this->size(); i++)
        {
            this->at(i)._scale=_scale;
            this->at(i)._xlen=_xlen;
            this->at(i)._ylen=_ylen;
            this->at(i)._zlen=_zlen;
        }
    }
    CPegArrayHole &find(std::string name)
    {
        static CPegArrayHole empty;
        for (size_t i = 0; i < this->size(); i++)
        {
            if (this->at(i).name == name)
                return this->at(i);
        }
        return empty;
    }

    /**
     * @brief creates a string that describes current pegarray name and centroid
     * 
     * @return std::string 
     */
    std::string toString()
    {
        std::stringstream ss;
        for (size_t i = 0; i < this->size(); i++)
            ss << this->at(i).name << ":" << this->at(i).state << ":"
               << this->at(i).offset.x() << ":"
               << this->at(i).offset.y() << ":"
               << this->at(i).offset.z() << "\n";
        return ss.str();
    }

    std::string toStringCentroid()
    {
        std::stringstream ss;
        for (size_t i = 0; i < this->size(); i++)
            ss << this->at(i).name << ":" 
               << conversion::dumpPoseRPY(this->at(i).centroid) << "\n";
        return ss.str();
    }
     /**
     * @brief set the centroid for the peg array. Used for future calculations.
     * 
     * @param centroid given xyz of the pegarray centroid (z is minimum of object)
     */
    void setCentroid(tf::Pose centroid)
    {
        this->centroid = centroid;
        for (size_t i = 0; i < this->size(); i++)
        {
            this->at(i).centroid = centroid;
        }
    }

    double xlen() { return _xlen*_scale; }
    double ylen() { return _ylen*_scale; }
    double zlen() { 
        return _zlen*_scale; 
        }
    void scale(double &scale)
    {
        _scale = scale;
        for (size_t i = 0; i < this->size(); i++)
        {
            this->at(i)._scale = _scale;
        }
    }
    /////////////////////////////////////////////////////////////
    std::string _name; // name of pegarray: base or
    double _xlen;     // constant x length
    double _ylen;     // constant x length
    double _zlen;     // constant x length
    double _scale;    // constant x length
    tf::Pose centroid;
};

typedef CPegHoleArray CPegArrayHoleOffsets ;

// struct CPegArray
// {
//     //static std::map<std::string, CPegArray *> names;
//     CPegArray(std::string name);
    
//     CPegArray &find(std::string name);

//     /**
//      * @brief determine if returned peg array is null/dummy
//      * 
//      * @param pegarray pegarray to test if null
//      * @return true  if null
//      * @return false  if actual peg array
//      */
//     bool isNullPegboard(CPegArray & pegarray)
//     {
//         return pegarray.name=="dummy.pegarray";
//     }

//     std::string toString()
//     {
//         std::stringstream ss;
//         ss << "Pegarray=" << name << "\n";
//         ss << "\t xlen=" << xlen << "\n";
//         ss << "\t ylen=" << ylen << "\n";
//         ss << "\t zlen=" << zlen << "\n";
//         ss << "\t scale=" << scale << "\n";
//         ss << "\t centroid=(" << centroid.x() << ","
//            << centroid.y() << ","
//            << centroid.z() << ")\n";
//         return ss.str();
//     }
//     std::string name;
//     double xlen;
//     double ylen;
//     double zlen;
//     double scale;
//     tf::Vector3 centroid;

//     /**
//      * @brief set the centroid for the peg array. Used for future calculations.
//      * 
//      * @param centroid given xyz of the pegarray centroid (z is minimum of object)
//      */
//     void setCentroid(tf::Vector3 centroid)
//     {
//         this->centroid = centroid;
//     }

//     /***
//      * topZarray returns the top Z based in the model, based on current centroid Z. 
//      * Use for example, determing top of a hole given a hole xyz offset in in pegarray,
//      * and then computing top of the hole.
//       **/
//     double topZpegarray()
//     {
//         return this->centroid.z() + zlen;
//     }

//     /**
//      * @brief getZ add zoffset using centroid Z and z length to top of pegboard. 
//      * Use for example, how far in positive Z direction is a peg when
//      * sitting in hole.
//      * 
//      * @param zoffset given offset to add to pegboard current location and offset
//      * @return double Z value given pegboard current centroid location
//      */
//     double gettopZoffset(double zoffset)
//     {
//         return centroid.z() + zlen + zoffset;
//     }
// };


// Global variables
extern CPegHoleArray pegs;
extern CPegHoleArray holes;
extern CPegArrayHoleOffsets supplybasearray;
extern CPegArrayHoleOffsets supplypegarray;
extern CPegArrayHoleOffsets destbasearray;
extern CPegArrayHoleOffsets destpegarray;

extern CrpiRobotParams params;
extern crpi_robot::CrpiGazebo robot;
extern tf::Quaternion qBend;

extern double dGzLengthPeg;       /// in meters - scale is 0.002
extern double zWrldMinPegArray; /// world coord in meters as reported by gazebo
extern double zSizePegArray;    /// in meters (scale is 0.002 mm->meters) twice as big
extern double zWrldMaxPegArray; /// zMinPegArray+zSizePegArray;  // in meters using calculator
extern double dDwellTime;       /// wait between moves or control actions (for settling)
extern double xfudge;
extern CPegArrayHole &emptyhole;
#endif
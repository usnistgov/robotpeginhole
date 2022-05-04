// robocalc.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Methods to use rosrun to invoke experiments
// Global debugging break flag
// BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
// DONT BREAK: rosrun test_gazebo_package test_gazebo_package rosbreak:=0

// Use Visual Code to attach debugger to looping break point
// rosrun test_gazebo_package test_gazebo_package rosbreak:=1

// Use clode w/wout debug break in visual code to log F/T experiments to
// see standard deviation. Possibly add ini/yaml file to add tests.
// rosrun test_gazebo_package test_gazebo_package rosbreak:=1  logft:=3

// No breaking for dead reckoning demonstration of ROS/Gazebo code
// rosrun test_gazebo_package test_gazebo_package rosbreak:=0 deadReckoning:=1

// catkin_make --only-pkg-with-deps robocalc
//Don't forget to switch back to building all packages when you are done
// catkin_make -DCATKIN_WHITELIST_PACKAGES=""
#include <iostream>
#include <robocalc/Hacks.h>
#include <robocalc/conversions.h>
#include <boost/algorithm/string.hpp>

#pragma comment(lib, "Ws2_32.lib")
#include <random>
#include <readline/readline.h>
#include <readline/history.h>
#include <tf/tf.h>

// INSTALLING READLINE 
// c:\opt\vcpkg>.\vcpkg.exe install readline
// BUILDING
// c:\opt\ros\noetic\crpi_test>catkin_make --only-pkg-with-deps robocalc

// RUNNING
//rosrun robocalc robocalc rosbreak:=0
// rpy 0  0.707107  0  0.707107

// Was looking for a 20o angle approach to hole.
// > rpy 0  0.707107  0  0.707107
// [0.000000,90.000000,0.000000]
// > q 0 110.0 0
// [0.000000,0.819152,0.000000,0.573576]
// > rpy 0  0.707107  0  0.707107
// [0.000000,90.000000,0.000000]
// > q 0 110.0 0
// [0.000000,0.819152,0.000000,0.573576]
// > q -20. 90.0 0
// [-0.122788,0.696364,0.122788,0.696364]
using namespace conversion;

inline double toDegree(double ang)
{
    return ang * 180.0 / M_PI;
}
inline double fromDegree(double ang)
{
    return ang / 180.0 * M_PI;
}



bool bBreak = 1;
bool bRadians = 1;

int main(int argc, char **argv)
{
    std::cout << "Hello World!\n";
    std::cout << "******** break = " << bBreak << "\n";

    // To break: rosrun test_gazebo_package test_gazebo_package rosbreak:=1
    std::vector<std::string> args;
    for (size_t i = 0; i < argc; ++i)
        args.push_back(argv[i]);

    bBreak = (bool)atoi(getCmdOption(args, "rosbreak:=", "1").c_str());

    // if no break then delay for Gazebo visual to finish loading.
    while (bBreak)
    {
        // sleep 1 second
        Sleep(1000);
    }

    try
    {
        char *buf;
        while ((buf = readline("> ")) != nullptr)
        {
            if (strlen(buf) > 0)
            {
                add_history(buf);
            }

            //printf("[%s]\n", buf);
            std::string msg(buf);
            boost::algorithm::trim(msg);

            // all input translated to lower case  - so case independent
            std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);
            if (msg.compare(0, strlen("quit"), "quit") == 0)
            {
                break;
            }
            if (msg.compare(0, strlen("help"), "help") == 0)
            {
                std::cout << "Commands:\n";
                std::cout << "\tquit - exits the calculator\n";
                std::cout << "\tdegrees - use degrees as I/O for rpy\n";
                std::cout << "\tradians - use radians as I/O for rpy\n";
                std::cout << "\trpy - calculate rpy from space delikmited quaternion x y z w floats\n";
                std::cout << "\tq - calculate quaternion from space delimited  roll pitch yaw floats\n";
                continue;
            }
            if (msg.compare(0, strlen("degrees"), "degrees") == 0)
            {
                bRadians = 0;
                continue;
            }
            if (msg.compare(0, strlen("radians"), "radians") == 0)
            {
                bRadians = 1;
                continue;
            }
            if (msg.compare(0, strlen("rpy"), "rpy") == 0)
            {
                msg = msg.erase(0, std::string("rpy ").size());
                boost::algorithm::trim(msg);
                double x, y, z, w;
                double roll, pitch, yaw;
                if (sscanf(msg.c_str(), "%lf %lf %lf %lf", &x, &y, &z, &w) == 4)
                {
                    tf::Quaternion q(x, y, z, w);
                    tf::Matrix3x3 m(q);
                    m.getRPY(roll, pitch, yaw);
                    //conversion::RotMatrix2Rpy(m, roll,pitch,yaw);
                    if (!bRadians)
                    {
                        roll = toDegree(roll);
                        pitch = toDegree(pitch);
                        yaw = toDegree(yaw);
                    }
                    printf("[%f,%f,%f]\n", roll, pitch, yaw);
                }
            }
            if (msg.compare(0, strlen("q "), "q ") == 0)
            {
                msg = msg.erase(0, std::string("q ").size());
                boost::algorithm::trim(msg);
                double roll, pitch, yaw;
                if (sscanf(msg.c_str(), "%lf %lf %lf", &roll, &pitch, &yaw) == 3)
                {
                    if (!bRadians)
                    {
                        roll = fromDegree(roll);
                        pitch = fromDegree(pitch);
                        yaw = fromDegree(yaw);
                    }
                    tf::Quaternion q;
                    q.setRPY(roll,pitch,yaw);
                    printf("[%f,%f,%f,%f]\n", q.x(), q.y(), q.z(), q.w());
                }
            }
            
            // readline malloc's a new buffer every time.
            free(buf);
        }
    }
    catch (...)
    {
        std::cerr << "Main program exception \n";
    }
    std::cout << "Goodbye World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

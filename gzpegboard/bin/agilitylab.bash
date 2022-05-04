#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash
root=`pwd`/..
export root


aprs=$root/Worlds/aprs-lab.world
gzver=7
export gzver
export aprs

p=`pwd`
export p
q=`pwd`/launch
r=$root/install/lib/gzrosrcs


# Definition of environment variables for bash
models=$root/gzdatabase/models
export models

plugins=$root/plugins$gzver
export plugins


libs=$root/install/lib
export libs


gzrosrcs=$root/install/lib/gzrosrcs
export gzrosrcs

aprs_objects=$root/install/lib/aprs_objects
export aprs_objects

# Kill any existing processes or can cause problems
pkill gzserver
pkill gzclient
pkill rosmaster

# Code to remove log files
#pushd .
#cd $gzrcs; find -type d -name "Log*" -prune; find -type d -name "Log*" -prune  -exec rm -rf {} \;
#popd

 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS\a\";source /opt/ros/kinetic/setup.bash;export myfolder=`pwd`;roslaunch roscore.launch ;exec bash"')

cmd+=( --tab   --working-directory="$p"  -e 'bash -c "printf \"\e]2;gazebo\a\";source /opt/ros/kinetic/setup.bash; LD_LIBRARY_PATH=/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-$gzver/plugins:$LD_LIBRARY_PATH;export LD_LIBRARY_PATH ; GAZEBO_PLUGIN_PATH=$plugins:/opt/ros/kinetic/lib:$GAZEBO_PLUGIN_PATH; export GAZEBO_PLUGIN_PATH;GAZEBO_MODEL_PATH=$models;export GAZEBO_MODEL_PATH; cd `pwd`; gazebo -s libgazebo_ros_api_plugin.so $aprs --verbose ; exec bash"')

"${cmd[@]}"


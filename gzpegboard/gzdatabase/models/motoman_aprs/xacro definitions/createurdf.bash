#!/bin/bash


p=`pwd`
cmd=( gnome-terminal )

# rosrun xacro xacro -vvv --inorder motoman_aprs.xacro  > motoman_aprs_xacro.urdf
# gz sdf -p motoman_aprs_xacro.urdf > motoman_aprs.sdf
# gz sdf -p motoman_aprs.urdf > motoman_aprs.sdf
# check_urdf motoman_aprs_xacro.urdf

cmd+=(--tab --working-directory="$p" -e 'bash -c "source /opt/ros/kinetic/setup.bash; rosrun xacro xacro --inorder motoman.xacro > motoman_aprs_xacro.urdf"')

cmd+=(--tab --working-directory="$p" -e 'bash -c "source /opt/ros/kinetic/setup.bash; rosrun xacro xacro --inorder motoman.xacro > motoman_aprs_xacro.urdf; gz sdf -p motoman_aprs_xacro.urdf > motoman_aprs.sdf"')

"${cmd[@]}"

#!/bin/bash
# prerequisite:
# sudo apt-get install screen
 
p=`pwd`
 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$p" -e 'bash -c "printf \"\e]2;Py Joint State Publisher\a\";source ../../devel/setup.bash;rosparam set /lrmate/source_list "['/lrmate/joint_states']";rosrun py_joint_state_pub  py_joint_state_pub ;exec bash"')


"${cmd[@]}"

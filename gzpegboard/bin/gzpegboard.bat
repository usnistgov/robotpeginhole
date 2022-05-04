:::::::::::::::::::::::::::::
:: Launch a taskboard world with Motoman robot
:: root
:: root/p
:: root/p/q
:: p=bin
:: q=bin/launch

:: assign root environment folder to parent folder
FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET root=%%x\..

:: assign p environment folder to bin folder
FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET p=%%x

:: assign q environment folder to bin\launch folder
set q=%p%\launch

:: Define the Gazebo world
set aprs=%root%\Worlds\aprs-lab.world
set models=%root%\gzdatabase\models
set plugins=%root%\pluginswin
set robot1_urdf=%q%\MotomanSia20d.urdf
set robot2_urdf=%q%\fanuc_lrmate200id.urdf

:: Gazebo
::LD_LIBRARY_PATH=/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-$gzver/plugins:$LD_LIBRARY_PATH;

:: Define location of model: type definitions in SDF
set GAZEBO_MODEL_PATH=
set GAZEBO_MODEL_PATH=%models%;%GAZEBO_MODEL_PATH%

set GAZEBO_PLUGIN_PATH=
set GAZEBO_PLUGIN_PATH=%plugins%;%GAZEBO_PLUGIN_PATH%


:: gazebo will require a rosrun
cd %q% 
set myfolder=%q%
roslaunch gazebo.launch 

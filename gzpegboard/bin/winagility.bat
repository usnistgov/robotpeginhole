

:: root
:: root/p
:: root/p/q
:: p=bin
:: q=bin/launch

::    ^^ this has to be bash, not /bin/sh, for arrays to work
:: run dos2unix ./runmultiterm.bash
FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET root=%%x\..



set aprs=%root%\Worlds\aprs-lab.world
echo aprs=%aprs%
::set gzver=7

FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET p=%%x

set q=%p%\launch


:: Definition of environment variables for gazebo
set models=%root%/gzdatabase/models
::set plugins=%root%/plugins%gzver%
set libs=%root%/install/lib


:: Kill any existing processes or can cause problems
taskkill /IM gzserver*
taskkill /IM gzclient*
taskkill /IM rosmaster*


:: Setup ROS
C:\Windows\System32\cmd.exe /k "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64&& set ChocolateyInstall=c:\opt\chocolatey&& c:\opt\ros\noetic\x64\setup.bat

:: ROS core
cd %q% 
set myfolder=%p%
roslaunch roscore.launch &

# Gazebo
cd %p%
$$LD_LIBRARY_PATH=/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-$gzver/plugins:$LD_LIBRARY_PATH;

::GAZEBO_PLUGIN_PATH=$plugins:/opt/ros/kinetic/lib:$GAZEBO_PLUGIN_PATH 
::export GAZEBO_PLUGIN_PATH
set GAZEBO_MODEL_PATH=%models%;%GAZEBO_MODEL_PATH%


gazebo -s libgazebo_ros_api_plugin.so %aprs% --verbose 






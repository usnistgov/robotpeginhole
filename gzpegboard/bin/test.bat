


FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET p=%%x
set q=%p%\launch
echo p=%p%
echo q=%q%

:: ROS core
cd %q% 
dir
set myfolder=%q%
roslaunch roscore.launch &




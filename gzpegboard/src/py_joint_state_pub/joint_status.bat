:: prerequisite:
:: sudo apt-get install screen
 
:: assign p environment folder to bin folder
FOR /f "usebackq tokens=*" %%x IN (`cd`) DO SET p=%%x

 

call ..\..\devel\setup.bat
::rosparam set /lrmate/source_list "['/lrmate/joint_states']"
rosrun py_joint_state_pub  py_joint_state_pub 




# Gazebo World Setup

----

  

This document gives a brief overview of how the worlds were built and then how to configure the World SDF file for Gazebo. Various models of the APRS laboratory are available, with configurable selections for versions, kitting models, robots, flags and the like.

# <a name="Modeling"></a>CAD Modeling of the World

The very first step before creating SDF files was to create all the meshes to be used in our APRS virtual environment. As you know, STL encodes geometry information only, as triangle meshes and does not include material specification. On the other hand, COLLADA can encode 3D geometry as well as physics, shaders, kinematics, etc which was the reason that it was a great choice if we wanted custom material properties such as textures besides geometry. For our APRS simulation, our choice was mainly COLLADA format and we have pretty much used the meshes in the following link for our environment:
http://data.nvision2.eecs.yorku.ca/3DGEMS/

For the walls and doors, we have created them in SketchUp and exported them as COLLADA file. For the camera sensor, we have used Gazebo’s built-in camera sensor model for capturing images. We had to set the pose of the sensor accordingly with respect to our camera model, 

    e.g.  <pose>0 -0.06 0 0 0 0</pose> 

in which the last three digits are for orientation. Then, we created SDF file in which we incorporated a sensor model and added the following code in the world file: 

    <model name="motoman_camera">
            <include>
                <static>true</static>
                <uri>model://motoman_camera_test</uri>
            </include>
        <pose frame=' ' >0.061138  -0.719002  2.005010  0  0  0</pose> 
    </model>

Last step was to create SDF files in Gazebo database for these virtual world items so that they could be spawned once the world file is launched.


# <a name="Configuration"></a>Configuration








To configure the Gazebo world, navigate to the ./World folder. You should see the following APRS lab world SDF definitions, as well as the configuration bash script world_setup.bash.


![Figure1](./images/gzworldsetup_image1.gif)



Double lick on the world_setup.bash file or invoke the world_setup.bash  scriptfrom the command line.  Once invoked you will get an simple yad GUI as shownin the figure  below to configure the aprs-lab.world  file used by the gzaprsros application.






![Figure2](./images/gzworldsetup_image2.gif)



The breakdown of the various configuration variable in shown in the table below.





<TABLE>
<TR>
<TD>Variable<BR></TD>
<TD>Description<BR></TD>
</TR>
<TR>
<TD>Physics Engine<BR></TD>
<TD>This selects the physics engine Gazebo will use. Its default is ODE, but other options are available but have been removed as they did not improve performance.<BR></TD>
</TR>
<TR>
<TD>Version<BR></TD>
<TD>There are 6 versions of the APRS lab to select from. Version 2 is a simple lightweight robot and kitting object model to lighten the simulation load on Gazebo. Version 6 can contain furniture, safety system, cameras, as part of the Gazebo world file.<BR></TD>
</TR>
<TR>
<TD>Debug<BR></TD>
<TD>The debug flag set to True will cause the Gazebo plugins to emit debugging messages.<BR></TD>
</TR>
<TR>
<TD>Motoman<BR></TD>
<TD>If True, the Motoman robot and assisted kitting models will be part of the world.<BR></TD>
</TR>
<TR>
<TD>Fanuc<BR></TD>
<TD>If True, the Fanuc robot and assisted kitting models will be part of the world.<BR></TD>
</TR>
<TR>
<TD>Safety<BR></TD>
<TD>If true, the safety curtain will be displayed.<BR></TD>
</TR>
<TR>
<TD>Camera<BR></TD>
<TD>If True, the cameras<BR></TD>
</TR>
<TR>
<TD>Conveyor<BR></TD>
<TD>If True, the conveyor model will be included in the model.<BR></TD>
</TR>
<TR>
<TD>Furniture<BR></TD>
<TD>If True, all the APRS lab furniture, PCs, monitors, etc. models will be included in the SDF world.<BR></TD>
</TR>
<TR>
<TD>ROS Plugins<BR></TD>
<TD>If true the gazebo-ros plugins for the conveyor, camera and safety system will be incorporated as part of the world SDF file.<BR></TD>
</TR>
<TR>
<TD>Gz Vision Sim<BR></TD>
<TD>If True, the vision simulator plugin will run which emits the Gazebo model for all kitting models on a given socket port (typically port 5002 but configurable).<BR></TD>
</TR>
</TABLE>








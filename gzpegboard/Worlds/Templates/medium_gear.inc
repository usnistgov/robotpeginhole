
#ifndef PAVEL
            <static>0</static>
            <link name="link1">
                <velocity_decay>
                    <angular>
             		.0005
             		</angular>
                </velocity_decay>
                <collision name="collision1">
                    <pose frame="">0 0 0 0 0 0</pose>
                    <geometry>
                        <mesh>
#ifdef HANDLES
                            <scale>1 1 1</scale>
                            <uri>model://gear_support/meshes/solid_medium_gear2.stl</uri>
#else
                            <scale>0.0254 0.0254 0.0254</scale>
                            <uri>model://gear_support/meshes/medium_gear.stl</uri>
#endif                    
                        </mesh>
                    </geometry>
                    <max_contacts>10</max_contacts>
#ifdef ODE
                    <surface>
                        <contact>
                            <ode>
                                <max_vel>0.1</max_vel>
                                <min_depth>0.001</min_depth>
                            </ode>
                        </contact>
                        <bounce />
                        <friction>
                            <ode>
                                <mu>1.0</mu>
                                <mu2>1.0</mu2>
                            </ode>
                        </friction>
                    </surface>
#endif
                </collision>
                <visual name="visual1">
                    <pose frame="">0 0 0 0 0 0</pose>
                    <geometry>
                        <mesh>
#ifdef HANDLES
                            <scale>1 1 1</scale>
                            <uri>model://gear_support/meshes/solid_medium_gear2.stl</uri>
#else
                            <scale>0.0254 0.0254 0.0254</scale>
                            <uri>model://gear_support/meshes/medium_gear.stl</uri>
#endif        
                        </mesh>
                    </geometry>
                    <material>
                        <script>
                            <name>Gazebo/Orange</name>
                            <uri>__default__</uri>
                        </script>
                    </material>
                </visual>
                <inertial>
                    <mass>0.02</mass>
                    <inertia>
                        <ixx>4.78233711e+00</ixx>
                        <ixy>2.06443170e-09</ixy>
                        <ixz>3.14256508e-00</ixz>
                        <iyy>4.78233728e+00</iyy>
                        <iyz>1.83990112e-08</iyz>
                        <izz>2.44284722e-01</izz>
                    </inertia>
                </inertial>
                <self_collide>0</self_collide>
                <kinematic> KINEMATIC </kinematic>
                <gravity>1</gravity>
            </link>

#else  

    <link name='gear_base'>
      <inertial>
        <mass>0.032</mass>
        <inertia>
          <ixx>0.000005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000005</iyy>
          <iyz>0</iyz>
          <izz>0.000007</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <pose frame=''>0 0 0.0098 0 -0 0</pose>
        <geometry>
          <mesh>
            <scale>0.0254 0.0254 0.0254</scale>
            <uri>model://meshes/medium_gear.stl</uri>
          </mesh>
        </geometry>
        <material>
          <lighting>1</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Orange</name>
          </script>
          <shader type='pixel'/>
        </material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>100</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.020491</radius>
            <length>0.018667</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>2000</mu>
              <mu2>2000</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>

    <link name='gear_axle'>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>2.6e-8</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>2.6e-8</iyy>
          <iyz>0</iyz>
          <izz>7.2e-9</izz>
        </inertia>
      </inertial>
      <pose>0 0 0.026909 0 -0 0</pose>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>100</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.003807</radius>
            <length>0.016514</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>2000</mu>
              <mu2>2000</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
            <torsional>
              <coefficient>1</coefficient>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>1e+06</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
            <bullet>
              <split_impulse>1</split_impulse>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+13</kp>
              <kd>1</kd>
            </bullet>
          </contact>
        </surface>
      </collision>
    </link>
    <joint name="joint1" type="fixed">
      <parent>gear_base</parent>
      <child>gear_axle</child>
    </joint>
    <static>0</static>
    <allow_auto_disable>1</allow_auto_disable>        
#endif

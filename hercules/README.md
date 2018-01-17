## Simulating your own robot in Gazebo
- We will publish control commands to the robot using the cmd_vel topic with geometry_msgs/Twist messages:
`rosmsg show geometry_msgs/Twist`
- Robot will report position using nav_msgs/Odometry messages:
`rosmsg show nav_msgs/Odometry`

### Part 1 - Setup
- Make a catkin_ws, if you do not have one: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Go to your catkin workspace: `cd ~/catkin_ws`
- Source your setup script: `source devel/setup.bash`
- Go to your src folder: `cd ~/catkin_ws/src`
- Clone Repo: `git clone -b jan-2018 https://github.com/arvpUofA/tutorials.git`

### Part 2 - URDF
- To simulate our robot in Gazebo and visualize in Rviz we will first build a model using URDF
- URDF is a xml format that allows you to design the physical characteristics of a robot http://wiki.ros.org/urdf
- Xacro (XML macro) allows you to simplify large URDF files using properties, macros, math expressions, conditional blocks and more. http://wiki.ros.org/xacro
- We will model a 4 wheel drive robot ( 1 chassis, 4 wheels )
    - In URDF you have links which define "parts" of the robot
    - To connect links you use joints which can be flexible or inflexible

![Alt text](pictures/joint-img.png?raw=true "Camera Calibration")
- http://wiki.ros.org/urdf/XML/joint

- Open up `hercules.urdf.xacro`, have already defined the robot dimensions using Xacro properties at the start of the file.
```
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="green">
        <color rgba="0.22 1.0 0.07 1"/>
      </material>
    </visual>
  </link>
```
- To see what this gives us we can preview it using the urdf_tutorial package. Run this command from with the tutorial directory: 
- `roslaunch urdf_tutorial  display.launch model:=src/tutorials/hercules/urdf/hercules.urdf.xacro`

- Next we can add the four wheels, since all wheels are the same except for position we can use a macro to define the geometry once and then use the macro 4 times. 
- The following is based macro off the wheel in the Clearpath jackal robot urdf: https://github.com/jackal/jackal/blob/indigo-devel/jackal_description/urdf/jackal.urdf.xacro 
```
  <xacro:macro name="wheel" params="prefix *joint_pose">
    <!-- Link -->
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black" />
      </visual>
    </link>

    <!-- Joint -->
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel" />
      <xacro:insert_block name="joint_pose" />
      <axis xyz="0 1 0" />
    </joint>
  </xacro:macro>
```
- Use the macro to create 4 wheels
```
  <xacro:wheel prefix="front_left">
    <origin xyz="${wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
  </xacro:wheel>
  <xacro:wheel prefix="front_right">
    <origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
  </xacro:wheel>
  <xacro:wheel prefix="back_left">
    <origin xyz="${-wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
  </xacro:wheel>
  <xacro:wheel prefix="back_right">
    <origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
  </xacro:wheel>
```
- Now check out the wheels in rviz: `roslaunch urdf_tutorial display.launch model:=src/tutorials/hercules/urdf/hercules.urdf.xacro`

### Part 3 - Simulating in Gazebo
- Now to simulate in Gazebo we need to add collsion and inertial information
- Collision geometry is often times simpler then the visual geometry, but in this case it will be exactly the same.
- Inertial tags will define the mass and moment of inertial for the link (can estimate using formulas here: https://en.wikipedia.org/wiki/List_of_moments_of_inertia) 

    - base_link (add under `<visual>` block in base_link link)
    ```
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.8"/>
      <inertia
        ixx="0.0029" ixy="0" ixz="0"
        iyy="0.0065" iyz="0"
        izz="0.0091"/>
    </inertial>
    ```
    - wheels (add under `<visual>` block in wheel macro `<link name="${prefix}_wheel">` block)
    ```
    <collision>
      <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.05"/>
      <inertia
        ixx="2.6326e-5" ixy="0" ixz="0"
        iyy="4.5156e-5" iyz="0"
        izz="2.6326e-5"/>
    </inertial>
    ```
- Finally add colors for in Gazebo:
    - base_link
    ```
    <gazebo reference="base_link">
      <material>Gazebo/Green</material>
    </gazebo>
    ```
    - wheels (goes inside wheel macro)
    ```
    <gazebo reference="${prefix}_wheel">
      <material>Gazebo/DarkGrey</material>
    </gazebo>
    ```
- Now launch Gazebo and spawn robot in it using launch file: `roslaunch hercules hercules_sim.launch`
    - Launch file does 3 things
        1. Loads robot model into parameter server
        2. Starts Gazebo with a empty world
        3. Uses spawn_urdf package to spawn robot in Gazebo
- Check out inertial and collsions in gazebo
![Alt text](pictures/hercules_gazebo.png?raw=true "Camera Calibration")
### Part 4 - Control in Gazebo
- We will use the diff_drive_controller package that allows use to control the robot in both simulation and real life.
- Checkout control.yaml which has the diff_drive_controller configuaration for a 4 wheeled skid steer drive robot. http://wiki.ros.org/diff_drive_controller
- Add gazebo control plugin definition to urdf file. This plugin allows the diff_drive_controller to control the simulated robot.
    ```
    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
      </plugin>
    </gazebo>
    ```
- Add transmission tag to wheel macro 
    - Describes relationship between joint and actuator
    - Will go into more detail next month
    ```
    <transmission name="${prefix}_wheel_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}_wheel_joint">
        <hardwareInterface>VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}_actuator">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
    ```
- Now launch control.launch `roslaunch hercules control.launch`
    -  Launch file does 4 things
        1. Loads diff_drive controller config into parameter server
        2. robot_state_publisher package is used to start publishing to tf topic
        3. Starts velociy controller and joint publisher
        4. Uses topic_tools to relay messages from cmd_vel topic to hercules_velocity_controller/cmd_vel

- And finally keyboard teleop: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
    - Install if you don't have it: `sudo apt-get install ros-kinetic-teleop-twist-keyboard`

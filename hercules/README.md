## Configuring ROS Navigation Stack

### Part 1 - Setup
- Make a catkin_ws, if you do not have one: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Go to your catkin workspace: `cd ~/catkin_ws`
- Source your setup script: `source devel/setup.bash`
- Go to your src folder: `cd ~/catkin_ws/src`
- Clone Repo: `git clone -b feb-2018 https://github.com/arvpUofA/tutorials.git`

### Part 2 - Review what we created last month
- Need to publish odometry tf frame, in control.yaml set:`enable_odom_tf: true`
- Launch simulator: `roslaunch hercules hercules_sim.launch`
    - Launch file does 3 things
        1. Loads robot model into parameter server
        2. Starts Gazebo with a empty world
        3. Uses spawn_urdf package to spawn robot in Gazebo
- Now launch control.launch `roslaunch hercules control.launch`
    -  Launch file does 4 things
        1. Loads diff_drive controller config into parameter server
        2. robot_state_publisher package is used to start publishing to tf topic
        3. Starts velociy controller and joint publisher
        4. Uses topic_tools to relay messages from cmd_vel topic to hercules_velocity_controller/cmd_vel
- Open up RVIZ : `rviz`
    - Set fixed frame to odom and add the robot model to the visualization
    - Not at tf and review the different coodinate frames

### Part 3 - Add laser sensor 
- The ROS Navigation stack requires laser scan data to work so we will add that sensor to our urdf model
- Add the following to the hercules.urdf.xacro:
```
  <link name="laser_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <joint name="laser_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="0 0 0.0425" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="laser_link"/>
  </joint>
```
- Above we defined a link for the laser scanner and a joint to connect it to the rest of the robot
- Now we need to add the gazebo plugin for simulating a laser scanner
```
<gazebo reference="laser_link">
    <sensor type="gpu_ray" name="laser">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gpu_laser" filename="libgazebo_ros_gpu_laser.so">
        <topicName>/scan</topicName>
        <frameName>laser_link</frameName>
      </plugin>
    </sensor>
</gazebo>
```
- Setting visualize to true will show laser scan in Gazebo
- Also launch control.launch and open up rviz and visualize the LaserScan 
### Part 4 - Configuring ROS Navigation Stack
- Take a look at the map in the maps directory, we will use this map to localize the robot in random_world.sdf
- The launch file amcl_demo.launch does 3 things:
    1. Launch map server to provided jackal_race map (retreived from: https://github.com/jackal/jackal/tree/indigo-devel/jackal_navigation/maps)
    2. Start amcl node: `roslaunch hercules amcl_demo.launch`
        a. Starts move_base + amcl with necessary config files (see below)

- The move_base requires some parameters
    - costmap_common_params.yaml contrains the params that are common to both the global and local costmaps
    - We also need specific config files for the global and local costmaps 
        - For our local costmap we use the map coordinate frame as the global fram. Some robots use the odom frame instead as it changes smoothly over time whereas the robots position in the map frame may have small jumps. But because the odometry drift is significant for this skid-steer-drive robot, I use the map frame instead.  

- Now launch simulator launch file, control launch file and then amcl_demo.launch
- Open up RVIZ
    - Add LaserScan, Map, Global Costmap, Local Cost Map, Pose array
    - Save config
- Drive the robot around so that AMCL can localize the robot. Once the pose array as converge on the position of the robot, try sending a nav goal using RVIZ.

- ![Alt text](pictures/hercules_gazebo.png?raw=true "Camera Calibration")
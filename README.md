## Using Turtlebot Simulator

Turtlebot Gazebo tutorial this is based off of: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it

##TODO: Add some images to clarify some of the steps

### Part 1 - Setup
- Install Gmapping package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-gmapping`
- Intall Turtlebot simulator package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-turtlebot-simulator`
- Install RViz launchers for convenience: `sudo apt-get install ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers`

### Part 2 - Launch Gazebo
- #TODO: Verify if `rosparam set /use_sim_time true` is necessary 
- `roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<path to catkin workspace>/catkin_ws/src/tutorials/turtlebot_tutorials/worlds/corridors.world`
  - http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it
- Launch teleop node: `roslaunch turtlebot_teleop keyboard_teleop.launch`

### Part 3 - Build map
- `roslaunch turtlebot_tutorials gmapping_demo.launch`
  - If you need to know the name of the coordinate frame for your robot:
    - `rosrun tf view_frames` creates a pdf showing the tf tree, or you can use view the messages being published to the /tf topic and use `child_frame_id`
- Launch RViz: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
  - To add image view:
      - Click Add -> Image 
      - Then click down arrow on Image from sidebar
      - Click on topic dropdown and select a image topic to display
- Drive around simulator using teleop node until the environment has been mapped out
- Save map: `rosrun map_server map_saver -f <your map name>`
- http://wiki.ros.org/gmapping
- http://openslam.org/gmapping.html
- https://github.com/turtlebot/turtlebot_apps/tree/indigo/turtlebot_navigation/launch/includes/gmapping

### Part 4 - Navigate Map
- Kill all processes and restart gazebo 
- Start amcl node: `roslaunch turtlebot_tutorials amcl_demo.launch map_file:=<path to map file you made in Part 3>`
- Start rviz again: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
- Follow step `2.3.3 Send Navigation Goal` from http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map

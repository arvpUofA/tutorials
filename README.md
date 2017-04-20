## Using Turtlebot Simulator

We will be following this tutorial but using our own local world file and copies of some of the launch files: http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it

### Part 1 - Setup
- git clone -b april-2017-turtlebot https://github.com/arvpUofA/tutorials.git
- Install Gmapping package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-gmapping`
- Intall Turtlebot simulator package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-turtlebot-simulator`
  - If having issues with Gazebo version: http://answers.ros.org/question/211291/ros-indigo-re-installation-problem/
- Install RViz launchers for convenience: `sudo apt-get install ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers`

### Part 2 - Launch Gazebo
- Launch Gazebo:
  -`roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<absolute path to worlds directory>/corridors.world`
  - If using Ubuntu 14.04 use corridors1404.world, or build your own model
- Launch teleop node: `roslaunch turtlebot_teleop keyboard_teleop.launch` and check and see if you can navigate the map

### Part 3 - Build map
- We will use the ROS gmapping package (http://wiki.ros.org/gmapping) which uses the external library with the same name (http://openslam.org/gmapping.html). GMapping uses Rao-Blackwellized particle filters to solve the SLAM problem and can be used to make grid maps.
- First launch the gmapping node:`roslaunch turtlebot_tutorials gmapping_demo.launch`
  - If you need to know the name of the coordinate frame for your robot:
    - `rosrun tf view_frames` creates a pdf showing the tf tree, or you can use view the messages being published to the /tf topic and use `child_frame_id`
- Next launch RViz to see map being built: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
  - To add image view:
      - Click Add -> Image 
      - Then click down arrow on Image from sidebar
      - Click on topic dropdown and select a image topic to display
- Drive around simulator using teleop node until the environment has been mapped out
- Save map: `rosrun map_server map_saver -f <your map name>`

### Part 4 - Navigate Map
- Now we will use the acml package (http://wiki.ros.org/amcl) which uses a particle filter for localization. And the move_base package (http://wiki.ros.org/move_base). The move_base node can be given a goal and will attempt to reach it using the ROS navigation stack. 
- To start you need to kill all processes from the previous parts and restart gazebo 
- Start amcl and move_base nodes: `roslaunch turtlebot_tutorials amcl_demo.launch map_file:=<absolute path to map file you made in Part 3>`
- Start rviz again: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
- To send a navigation goal for the turtlebot follow section `2.3 RViz` from: http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map

## A simple State Machine using ROS Smach II   
**Version 0.2**  

**Reference**  
[ROS Smach](http://wiki.ros.org/smach)

**Scenario**  
Our robot is looping over a task that involves serching for targets, grabbing objects and dropping them at a different place, then back to starting point and repeat its lonely mission.  
We use a simple state machine to handles transitions between tasks.  

**States**  
- Searching  
- Grabbing  
- Going to a new location  
- Dropping  
- Back to Searching  

**Todo**  
Transitions are simplified and have been connected from last month's demo. We removed those
dummy counter and will be plug in real data time.

(So I "fixed" the code and saved our robot from an endless job :P)  
Stretches: Instead of creating a new state, we will be reusing one of the previous state.  

1. Download the code from github repo
2. Open up your favourite editor  
3. Read over and understand the code  
4. Start your building
5. To run the code, there are two ways of doing so. One is download turtlebot packages and run them your own. The other way, and the recommended way, is to locate ROS Master that is running on our spearker's laptop, where all the turtlebot packages are configured and running, and then launch your own testing scripts. A template launch file can be found under launch folder.  
6. If you would like to download your own Turtlebot code, please refer to this website: [Turtlebot](http://wiki.ros.org/Robots/TurtleBot). The installation instruction is with ROS Indigo, available both in package and source installation. If you are using any release after Indigo and would love to install from source, I am working on a bash script to pull all of them.  

**Testing**  
1. Connect to host ROS core using `export ROS_MASTER_URI=http://***:11311`  
2. Launch your scriptusing `roslaunch pioneer_bot local_script.launch`  
  

## Setup an AR Tag camera with the potential to be visualized

We will be following this tutorial but using our own local usb camera by default in our laptops: 
http://wiki.ros.org/ar_track_alvar

### Part 1 - Setup
- `cd ~/catkin_ws`
- `source devel/setup.bash`
- `cd ~/catkin_ws/src`
- Clone Repo: `git clone -b oct-2017-ARTags https://github.com/arvpUofA/tutorials.git`
- Install usb-cam package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-usb-cam`
- Intall ar-track-alvar package: `sudo apt-get install ros-<YOUR VERSION OF ROS>-ar-track-alvar`

### Part 2 - Launch usb_cam
- Launch usb_cam:
  - `roslaunch usb_cam usb_cam-test.launch`

### Part 3 - Calibrate the usb_cam
- We will use the ROS camera_calibration package (http://wiki.ros.org/camera_calibration) which follows the monocular calibration tutoriual with a few modifications (http://openslam.org/camera_calibration/tutorials/MonocularCalibration). 
- First run the camera calibration node:`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0256 image:=/usb_cam/image_raw camera:=/usb_cam`
  - Take your default camera calibration board checker board and move it about until all bars are green:
![Alt text](ar_tag/pictures/mono0.png?raw=true "Camera Calibration")
  - Once archieved, click "Calibrate" and wait for calibratioon to process.
  - Then click "Save" and "Commit".
![Alt text](ar_tag/pictures/mono1.png?raw=true "Camera Calibration")

### Part 4 - Launch ar_track_alvar
- Now we will use the acml package (http://wiki.ros.org/amcl) which uses a particle filter for localization within a known map. In addition, the move_base package will be used (http://wiki.ros.org/move_base). The move_base node can be given a goal and will attempt to reach it using the ROS navigation stack. 
- To start you need to kill all processes from the previous parts and restart gazebo 
- Start amcl and move_base nodes: `roslaunch turtlebot_tutorials amcl_demo.launch map_file:=<absolute path to map file you made in Part 3>`
- Start rviz again: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
- To send a navigation goal for the turtlebot follow section `2.3 RViz` from: http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map
- If your robot is not moving even after you send it a navigation goal, make sure you have shutdown the teleop node
![Alt text](turtlebot_tutorials/pictures/rviz3.png?raw=true "Rviz acml in action")

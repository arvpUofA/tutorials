## Setup an AR Tag camera with the potential to be visualized

We will be following this tutorial but using our own local usb camera by default in our laptops: 
http://wiki.ros.org/ar_track_alvar

### Part 1 - Setup
- Make a catkin_ws, if you do not have one: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Go to your catkin workspace: `cd ~/catkin_ws`
- Setup your bash path: `source devel/setup.bash`
- Go to your src folder: `cd ~/catkin_ws/src`
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
- Now we will will modify our launch file to accept our new rostopics we have just setup, `/usb_cam/image_raw`, `/usb_cam/camera_info`, `/usb_cam`
- To start your launch file, `roslaunch tutorials indiv_no_kinetic` 
- Now, in a new terminal, ensure your node is working and ARTag positions are being published `rostopic echo /ar_pose_marker`

### Part 5 - Take home assignment, tf broadcaster
- Start rviz: `rviz`
- Add a image node and select your `/usb_cam/image_raw` topic and view your camera feed.
- Add a Marker
- Now go to http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29 and setup a tf broadcaster to map your published coordinate space to the camera image.

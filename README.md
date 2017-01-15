## Using ROS with Gazebo Simulator Part 2

Presentation: https://docs.google.com/presentation/d/1sLNUqFyN1WcnksPeObNLntVeC7U2imrrR01AVJhllqs/edit?usp=sharing

### Part 1 - Setup
- Add package under src folder in catkin workspace: `git clone -b ros-edmonton-jan https://github.com/arvpUofA/tutorials.git`
- Run `catkin_make` from root of catkin workspace
- TODO: Copy model file to ~/.gazebo/models 

### Part 2 - Adding camera to world model
- TODO

### Part 3 - OpenCV 
- TODO: replace everything below with OpenCV content 
- Open up the python script `scripts/pioneer_bot.py`
- Add a publisher and subscriber to PioneerBot constructor:
``` python
  self.odometry_sub = rospy.Subscriber("pioneer2dx/odom", Odometry, self.odometry_callback)
  self.velocity_pub = rospy.Publisher("pioneer2dx/cmd_vel", Twist, queue_size=1) 
```
- Take a look at the Odometry message:
```
  nav_msgs/Odometry.msg
  # This represents an estimate of a position and velocity in free space.  
  # The pose in this message should be specified in the coordinate frame given by header.frame_id.
  # The twist in this message should be specified in the coordinate frame given by the child_frame_id
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```

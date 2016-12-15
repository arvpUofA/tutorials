## Using ROS with Gazebo Simulator 

Presentation: https://docs.google.com/presentation/d/1sLNUqFyN1WcnksPeObNLntVeC7U2imrrR01AVJhllqs/edit?usp=sharing

### Part 1 - Setup
- Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace, or skip if you already have one 
- Add package under src folder in catkin workspace: `git clone -b ros-edmonton-dec https://github.com/arvpUofA/tutorials.git`
- Run `catkin_make` from root of catkin workspace
- Start ROS master in terminal: `roscore`
- In new terminal run start simulator: `rosrun gazebo_ros gazebo -file model/pioneer2dx_ros.world`
- View topics being published by differential drive plugin: `rostopic list`
- Watch odometry topic: `rostopic echo /pioneer2dx/odom`
- Control robot by publishing to `/pioneer2dx/cmd_vel` topic: `rosrun pioneer_bot teleop_key.py`

### Part 2 - Subscriber Callback
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
- Update robot position and orientation when Odometry Subscriber callback is called:
```python
  orientation = data.pose.pose.orientation
  position = data.pose.pose.position

  self.x = position.x
  self.y = position.y

  quaternion = (orientation.x,orientation.y,orientation.z,orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  self.orientation = euler[2] if euler[2] > 0 else euler[2] + 2*math.pi
```
### Part 3 - Move to target function
- Add PD controller code to follow path:
```python
  rx =  self.x - p1[0]
  ry =  self.y - p1[1]
  
  u  = (rx * dx + ry * dy) / (dx * dx + dy * dy)
  error = (ry * dx - rx * dy) / math.sqrt(dx * dx + dy * dy)

  diff_error = error - last_error
  last_error = error

  steer = - error * self.P - diff_error * self.D
  # Note with Gazebo 2, a bug in the ROS plugin means you have to pass the negative of the steering commands
```
- Play with P and D constants until you find something that works 
- Test drive_to_goal funtion in main 

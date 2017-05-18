## A simple State Machine using ROS Smach  

**Reference**  
[ROS Smach](http://wiki.ros.org/smach)

**Scenario**  
Our robot is looping over a task that involves serching for targets, grabbing objects and dropping them at a different place, then back to searching.  
We use a simple state machine that handles transitions between tasks.  

**Tasks**  
- Searching  
- Grabbing objects  
- Going to a different place  
- Dropping objects  
- Back to Searching  

**Todo**  
Code has provided transitions between Searching to Grabbing, Grabbing to Dropping, and Dropping to Back to Searching. Today's tryout is to simply add one more state between Grabbing and Dropping, that is, to move to a different place.  
  
But before that, if you run the code, you will find the robot is doing the tasks over and over again. Our robot gets tired(wait, a robot gets tired?), let's try give it a break but changing the final state transition back to its initial state.  After that, you can start giving it a new state, or make it back to work loop maybe.  
  
Feel free to name your state and design your own state I/O.  

1. Download the code from github repo 
2. Open up your terminal  
3. Go to your catkin_ws and start `roscore`  
4. Run the code using `rosrun pioneer_bot state.py`  
5. Open up your favourite editor  
6. Read over and understand the code  
7. Here we go, start your building.

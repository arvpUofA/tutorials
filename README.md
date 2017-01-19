## Using ROS Gazebo Camera Plugin

### Part 1 - Setup
- Add package under src folder in catkin workspace: `git clone -b ros-edmonton-jan https://github.com/arvpUofA/tutorials.git`
- Run `catkin_make` from root of catkin workspace
- `source devel/setup.bash`
- Copy model file from pioneer_bot package to gazebo models folder: `cp -rf model/monocular_camera ~/.gazebo/models`

### Part 2 - Adding camera to world model
- Open simulator model file in pioneer_bot package: `gedit model/pioneer2dx_ros.world`
- Add camera model below plugin tag for differential drive controller: 
``` xml
  <include>
    <uri>model://monocular_camera</uri>
    <pose>0.2 0 0.2 0 0 0</pose>
  </include>
```
- Add joint to connect pioneer_bot chassis to the camera
``` xml
      <joint name="camera_joint" type="revolute">
        <pose>0 0 0 0 0 0</pose>
    	<axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
        <child>monocular_camera::link</child>
        <parent>chassis</parent>
      </joint>
```
- See documation on sdf format for joint: http://sdformat.org/spec?elem=joint
- Start gazebo from pioneer_bot package directory: `rosrun gazebo_ros gazebo model/pioneer2dx_ros.world`
- Start rqt and open image view plugin: `rqt` select /camera/rgb/image_raw topic



### Part 3 - OpenCV 
- Open up the python script `scripts/pioneer_bot.py`
- Code1: Convert image to grayscale and use OpenCV HoughCircles function to find circles, http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles#cv2.HoughCircles
``` python
  gray_img = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
  circles = cv2.HoughCircles(gray_img,cv2.cv.CV_HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)
    
```
- Code2: If a circle is found move towards it
```python
       if(circles is not None):
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                    if(i[2]>80):
                       self.done = True
                       self.move(0, 0)
                       break
    	        # draw the outer circle
    		    cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
    		    self.move((rows/2-i[0]) * 0.01, 1)
    		    break
       else:
            if (self.done == False):
                   self.move(1, 0)
```
- Run node: `rosrun pioneer_bot pioneer_bot`

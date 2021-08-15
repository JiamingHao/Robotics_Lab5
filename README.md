# COMS W4733 Robotics Lab 5

## Usage
####  For the following steps with * attached, if  you can directly launch turulebot and map using commands in step 4, then steps 2 to 3 can be skipped.  If there is namespace warning, the solution can be found from: 		  [https://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/]
#### 1. Extract lab5_UNI1_UNI2.tar.gz:


#### *2. Put the whole followbot folder under catkin_ws/src directory, run the following command:
```shell
$ catkin_make
```
#### *3. Run the following command:
```shell
$ roscore
```
#### 4. In a separate terminal, launch turtlebot and map:
To launch turtlebot and map for part 1:
```shell
$ roslaunch followbot launch.launch
```
To launch turtlebot and map for part 2:
```shell
$ ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```
To launch turtlebot and map for part 3:
```shell
$ ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```
#### 5. In another separate terminal, run scripts for different parts:
For part 1:
```shell
$ python follower_p1.py
```
For part 2:
```shell
$ python follower_p2.py
```
For part 3:
```shell
$ python follower_p3.py
```
## Methods
```python
def image_callback(self, msg):
  """
  Image handler functions implemented in all of the three scripts for three different parts.
  Take the picture from rgb camera and do processing, after that, control the robot accordingly.
  Implemention is different in details for different parts; for part 1 and 2, mainly color detection,
  for part 3, color detection and shape detection.
  """
```

## Video
### Part 1
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/3tzIc_c79SM/0.jpg)](https://youtu.be/3tzIc_c79SM)

### Part 2
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/R2iSOkB_qJc/0.jpg)](https://youtu.be/R2iSOkB_qJc)

### Part 3
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/BAC8npaArKE/0.jpg)](https://youtu.be/BAC8npaArKE)

## Others
For some reason we are not aware of, we get slightly different results for the two of us. The videos we included above are the results we get when running the programs on Yueen Ma(ym2745)'s computer.

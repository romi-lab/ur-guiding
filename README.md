# UR Guiding

## Overview - A handy device for controlling Universal Robot
To intuitively control UR, we developed a device with AR markers and an IMU unit.
The device output absolute pose with respect to **base_link**,
where the position information is givne by AR marker and the orientation is defined by the IMU sensor.
The transformation from camera to base is computed on a real-time basis.
The setup is illustrated below.

<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/device.png" width="400" alt=""> <img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/setup.png" width="400" alt="">

The trjecotry can be recorded with 6 DOF:

<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/traj6.gif" width="800" alt="">

And this can be stored, or it can be used for a real-time follwing:

<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/following.gif" width="800" alt="">  

I further developed it for 3d drawing and robotic welding path planning (check below subsections for detail). 
The whole program is developed in ROS kinetic, most scripts are in python.

## How is it made?
### AR marker
We obtained the position of AR marker using the package [ar_track_alvar](http://wiki.ros.org/ar_track_alvar).
In our lab, there are two cameras, one kinect mounted at the top and another realsense mounted at the end-effector of robot manipulator.
Both can be used for tracking the AR marker and record the trajectory.

### IMU
We obtained orientation information form the sensor in ROS. For detail of the sensor and its usage in ROS can be found at [jy901-imu-ros](https://github.com/maggielovedd/jy901-imu-ros).

### The algorithm
<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/algorithm.png" width="800" alt="">

## Filter the path
The motion and detection and introduce considerable noise. Therefore, I used voxel grid for filtering the nearby points.
As shown below, the yellow represent the original trajectory and red is the filtered result. This is done online.

<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/voxel.gif" width="800" alt="">

## Explaination of the nodes

Node | Function
------------ | -------------
kinect_pen_trajectory.py | use kinect camera to detect AR and generate trajectory
ar_marker_transformation.py | obtain real-time transformation from realsense camera to base_link
rs_pen_trajectory.py  | use realsense camera to detect AR and generate trajectory
trajectory_execution.py | execution trajectory using UR
ur_imu.py | control the UR by orientation only
voxel.py  | voxel downsampling the trajectory
filtered_trajectory.py | voxel downsampling the trajectory and remap it to 3d space

## Command
To visualise the trajectory:  

```roslaunch ur-guiding trajectory_visual.launch```

To filter the trajectory using voxel grid:  

```roslaunch ur-guiding filtered_trajectory.launch``` 

## Application

### 3d drawing
I packed the tip trajecotry of the device as the marker array and visualised in Rviz:

<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/drawing.gif" width="800" alt="">  

### robotic welding path planning

Same principle, we record the path and then executed it by the manipulator.
Three shapes are showed for illustration.

#### Plane
<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/line.gif" width="800" alt="">  

#### Tube
<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/tube.gif" width="800" alt="">  

#### Y-shape
<img src="https://github.com/romi-lab/ur-guiding/blob/main/demo/yshape.gif" width="800" alt="">  

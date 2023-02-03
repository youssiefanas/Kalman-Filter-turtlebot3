# Kalman-Filter-turtlebot3
Kalman Filter Esimation of TurtleBot3 Rotation. Sensor fusion technique will be developed for indoor localization of Turtlebot 3 mobile robot. The proposed sensor fusion method combines the measurements made by odometry of the robot and the acceleration measured by an onboard IMU to get the true state of the robot.


`git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git`  

-To launch the launch file:  
  - `roscore`  
	- `export TURTLEBOT3_MODEL=burger`  
	- `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
	- `roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch`  
  
  ```
  rosrun turtlebot_kf turtlebot3_kf.py 
  rosrun turtlebot_kf noisy_odom.py 
  
  ```
  
-IMU readings:  
	- rostopic echo /imu  
  
-To use turtle_teleop_key:  
	- `rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/cmd_vel`  

- using rqt_plot: 

```
	/imu/linear_acceleration/x  
	/imu/linear_acceleration/y  
	/imu/linear_acceleration/z  
  ```
- To Reset simulation  
	`rosservice call /gazebo/reset_simulation`  

- Circular path with linear/x and angular/z from `rqt`:  
	-`/cmd_vel /geometry_msgs/Twist: linear.x = 0.7 angular.z = 3`  

```				 
rostopic info /odom
rosmsg show nav_msgs/Odometry
```

using rqt_plot:  
```
    /noisy_state/data[0]  
	  /noisy_state/data[1] 
    /filtered_heading/data[0]
    /filtered_heading/data[1]
```  

https://user-images.githubusercontent.com/121443735/215361222-f8197abb-bffa-41ac-a061-888dc420fe0f.mp4



https://user-images.githubusercontent.com/121443735/215361235-d52279f4-0c48-444a-85ed-f2713aa6eceb.mp4

![Odom Noisy Plot](https://user-images.githubusercontent.com/121443735/215361236-a8056939-05b7-4bb4-90f8-d5482c47ccce.png)  


https://user-images.githubusercontent.com/121443735/215361242-fd45042d-ea6d-4df9-891f-f57a92ed9d0a.mp4

![Filtered Plot](https://user-images.githubusercontent.com/121443735/215361245-f35fb1be-33c1-4cff-ba9a-b9c2c912fb51.png)  

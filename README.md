# ROBOTICS

## Checkpoint Final Instructions 

### Prerequisites

This project uses ROS to interface with the V-REP simulator, so linux with ROS installed is required. The code has been tested to work in ubuntu 16.04 with ROS Kinetic Kame. ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation "Install ROS Kinetic Kame"). 

### Building and Running the code

Open terminal and clone the codebase.

```
git clone https://gitlab.engr.illinois.edu/stier2/robotics.git
```

Navigate to the catkin workspace folder, build the robot controller code, make it executable, and source the workspace.

```
cd robotics/catkin_ws
catkin_make
chmod +x src/robot_controller/scripts/controller_final_task.py
chmod +x src/robot_navigation/scripts/sendtime.py
source devel/setup.bash
```

Open two more terminals. In each of them, navigate to the same workspace and source the workspace.

```
source devel/setup.bash
```

You should now have three terminals open. In one of them, start a roscore.

```
roscore
```

In another terminal, navigate to your directory containing V-REP and launch it.

```
./vrep.sh
```


### Demonstrating SLAM 

First install the hector slam package. 

```
sudo apt-get install ros-kinetic-hector-slam
```
Open the scene cp_final_slam.ttt

Set execution time
```
rosrun robot_navigation sendtime.py
```
Run the hector slam method in another terminal.

```
roslaunch robot_slam hector.launch
```
 
RViz will be launched and will show a visualization of the of the map.

The robot also responds to directional keyboard input (up, down, left, right, and space to stop) if you click on the scene in V-REP. As the LIDAR scans more of the environment, the map will continue to be updated in the RViz visualization.

Save the map
```
rosrun map_server map_saver -f mymap
```

### Demonstrating Path Generating
First install relevant package
```
sudo apt-get install ros-kinetic-navigation
```

Launch the navigation in the terminal 
```
roslaunch robot_navigation amcl_demo_lidar_rviz.launch
```

Then use the mouse to select the initial pose and goal in Rviz.

### Demonstrating Final Task Execution

Open the scene cp_final_task.ttt and run the control code in the terminal.

```
rosrun robot_controller controller_final_task.py
```

Press play in V-REP and watch as the robot travels to the block, picks it up, travels to the table, and releases the block onto the table.



## Checkpoint 4 Instructions

### Prerequisites

This project uses ROS to interface with the V-REP simulator, so linux with ROS installed is required. The code has been tested to work in ubuntu 16.04 with ROS Kinetic Kame. ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation "Install ROS Kinetic Kame"). 

### Building and Running control code

Open terminal and clone the codebase.

```
git clone https://gitlab.engr.illinois.edu/stier2/robotics.git
```

Navigate to the catkin workspace folder, build the robot controller code, make it executable, and source the workspace.

```
cd robotics/catkin_ws
catkin_make
chmod +x src/robot_controller/scripts/controller_cp4.py
chmod +x src/robot_navigation/scripts/sendtime.py
source devel/setup.bash
```

Open two more terminals. In each of them, navigate to the same workspace and source the workspace.

```
source devel/setup.bash
```

You should now have three terminals open. In one of them, start a roscore.

```
roscore
```

In another terminal, navigate to your directory containing V-REP and launch it.

```
./vrep.sh
```

Once V-REP has launched, open the scene located at `robotics/vrep_scenes/cp4_InvKin_SLAM.ttt` and press the blue play button on the top toolbar.


#### Demonstrating Inverse Kinematics

In another terminal, run the robot control code.

```
rosrun robot_controller controller_cp4.py
```

The controller will calculate a set of joint angles that will place the end effector in a predefined configuration relative to the base arm. The terminal output will show the desired transformation, whether or not the inverse kinematics were successful, the number of times inverse kinematics was attempted with different initial conditions, the joint angles found by the inverse kinematics, and the calculated end-effector transformation based on the inverse kinematics joint angles. Below is example teminal output for a successful inverse kinematics attempt.

```
('The desired transformation is:', array([[ 0.  ,  0.  ,  1.  ,  0.4 ],
       [ 0.  , -1.  ,  0.  , -0.35],
       [ 1.  ,  0.  ,  0.  ,  0.1 ],
       [ 0.  ,  0.  ,  0.  ,  1.  ]]))
Inverse kinematics convergence success!
('attempt number', 0)
Joint angles found through inverse kinematics
[[-33.9111523951], [63.9263325933], [-18.3392242276], [37.6650969544], [-20.4203522483], [61.9074235394]]
Expected end effector transformation given the inverse kinematics joint angles
[[ -2.68748595e-15  -1.60982339e-15   1.00000000e+00   4.00000000e-01]
 [  5.59225048e-17  -1.00000000e+00  -1.88737914e-15  -3.50000000e-01]
 [  1.00000000e+00   2.59130109e-16   2.64044832e-15   9.99999999e-02]
 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
```

The accuracy of the solution can be verified by inspecting the output of the console in V-REP which prints the position of the end-effector of with respect to the robot base. Below is example output corresponding to the previous example.

```
end-effector position w.r.t. robot base
{0.39868101477623, -0.34881871938705, 0.098395079374313}
```


### Demonstrating SLAM 

First install the hector slam package. 

```
sudo apt-get install ros-kinetic-hector-slam
```
Open the scene cp_final_slam.ttt

Set execution time
```
rosrun robot_navigation sendtime.py
```
Run the hector slam method in another terminal.

```
roslaunch robot_slam hector.launch
```
 
RViz will be launched and will show a visualization of the of the map.

The robot also responds to directional keyboard input (up, down, left, right, and space to stop) if you click on the scene in V-REP. As the LIDAR scans more of the environment, the map will continue to be updated in the RViz visualization.

## Checkpoint 3 Instructions

### Prerequisites

This project uses ROS to interface with the V-REP simulator, so linux with ROS installed is required. The code has been tested to work in ubuntu 16.04 with ROS Kinetic Kame. ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation "Install ROS Kinetic Kame"). 

### Building and Running control code

Open terminal and clone the codebase.

```
git clone https://gitlab.engr.illinois.edu/stier2/robotics.git
```

Navigate to the catkin workspace folder, build the robot controller code, make it executable, and source the workspace.

```
cd robotics/catkin_ws
catkin_make
chmod +x src/robot_controller/scripts/controller_cp3.py
source devel/setup.bash
```

Open two more terminals. In each of them, navigate to the same workspace and source the workspace.

```
source devel/setup.bash
```

You should now have three terminals open. In one of them, start a roscore.

```
roscore
```

In another terminal, navigate to your directory containing V-REP and launch it.

```
./vrep.sh
```

Once V-REP has launched, open the scene located at `robotics/vrep_scenes/cp3_forward_kinematics.ttt` and press the blue play button on the top toolbar.

In another terminal, run the robot control code.

```
rosrun robot_controller controller_cp3.py
```

The robot should now start moving. As it did in checkpoint 2, it will move forward and backward, changing direction when the front or rear proximity sensor detects an object. The UR3 arm joints will oscillate between positive and negative pi/10. The terminal will output the transofmation matrix, T, that describes the transformation from the tip of the robot end effector to the base of the robotic arm, thus demonstrating forward kinematics.

As a reference, a pdf (ECE_470_Project_Update_3___Forward_Kinematics.pdf) detailing the deivtion of the forward kinematics used within this repository is included in the same directory as this readme file.


## Checkpoint 2 Instructions

### Prerequisites

This project uses ROS to interface with the V-REP simulator, so linux with ROS installed is required. The code has been tested to work in ubuntu 16.04 with ROS Kinetic Kame. ROS installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation "Install ROS Kinetic Kame"). 

### Building and Running control code

Open terminal and clone the codebase.

```
git clone https://gitlab.engr.illinois.edu/stier2/robotics.git
```

Navigate to the catkin workspace folder, build the robot controller code, make it executable, and source the workspace.

```
cd robotics/catkin_ws
catkin_make
chmod +x src/robot_controller/scripts/controller_cp2.py
source devel/setup.bash
```

Open three more terminals. In each of them, navigate to the same workspace and source the workspace.

```
source devel/setup.bash
```

You should now have four terminals open. In one of them, start a roscore.

```
roscore
```

In another terminal, navigate to your directory containing V-REP and launch it.

```
./vrep.sh
```

Once V-REP has launched, open the scene located at `robotics/vrep_scenes/cp2_interface.ttt` and press the blue play button on the top toolbar.

In another terminal, run the robot control code.

```
rosrun robot_controller controller_cp2.py
```

The robot should now start moving. It will move forward and backward, changing direction when the front or rear proximity sensor detects an object. The UR3 arm joints will oscillate between positive and negative pi/10. During simulation, sensor readings for front and rear proximity sensors will each be logged to a separate csv. In another terminal, use `ls` in the catin_ws directory to see a list of generated csv files. To view sensor and actuator readings in real time, find the available topics with the following:

```
rostopic list
```
The following topics contain the sensor readings received by this node.

```
/sensor_UR3_joint_position_1
/sensor_UR3_joint_position_2
/sensor_UR3_joint_position_3
/sensor_UR3_joint_position_4
/sensor_UR3_joint_position_5
/sensor_UR3_joint_position_6
/sensor_arm_color
/sensor_arm_depth
/sensor_front_color
/sensor_front_depth
/sensor_kinetic_color
/sensor_kinetic_depth
/sensor_trigger_front
/sensor_trigger_rear
/sensor_lidar_scan
```

The following topics contain the actuator control commands sent from this node to the V-REP simulator.

```
/actuator_UR3_joint_target_1
/actuator_UR3_joint_target_2
/actuator_UR3_joint_target_3
/actuator_UR3_joint_target_4
/actuator_UR3_joint_target_5
/actuator_UR3_joint_target_6
/actuator_motor_speed_left
/actuator_motor_speed_rigth
```


Then print values being published to one of the sensor or actuator topics as shown by the following example:

```
rostopic echo /sensor_UR3_joint_position_5
```

RBG video from any of the /sensor_xyz_color topics can also be seen in terminal by running

```
rosrun image_view image_view image:=/sensor_kinect_color
```

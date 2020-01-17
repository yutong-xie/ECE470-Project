#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import forward_kinematics
import inverse_kinematics
import rotations
import numpy as np
from state import ArmState, BaseState, RobotState
from scipy.linalg import   expm, sinm, cosm, logm

import os.path
from os import path
import csv


#class for controlling the robot
class RobotController:

    #initialize class instance
    def __init__(self):    
    

        #state variables
        self.robot_state = RobotState.INITIALIZING
        self.arm_state = ArmState.STATIONARY
        self.base_state = BaseState.STATIONARY


        #configuration variables
        self.robot_motor_speed = 3.0
        self.robot_direction = 1.0

        #declare the publishers, passing in the topic to be published to
        self.pub_rms = rospy.Publisher('actuator_motor_speed_right', Float32, queue_size=10)
        self.pub_lms = rospy.Publisher('actuator_motor_speed_left', Float32, queue_size=10)
        self.pub_UR3_joint_target_1 = rospy.Publisher('actuator_UR3_joint_target_1', Float32, queue_size=10)
        self.pub_UR3_joint_target_2 = rospy.Publisher('actuator_UR3_joint_target_2', Float32, queue_size=10)
        self.pub_UR3_joint_target_3 = rospy.Publisher('actuator_UR3_joint_target_3', Float32, queue_size=10)
        self.pub_UR3_joint_target_4 = rospy.Publisher('actuator_UR3_joint_target_4', Float32, queue_size=10)
        self.pub_UR3_joint_target_5 = rospy.Publisher('actuator_UR3_joint_target_5', Float32, queue_size=10)
        self.pub_UR3_joint_target_6 = rospy.Publisher('actuator_UR3_joint_target_6', Float32, queue_size=10)

        self.pub_suction = rospy.Publisher('actuator_suction_toggle', Bool, queue_size=10)


        #initialize node with the given name
        rospy.init_node('robot_controller', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        #declare the subscribers, passing in the topic and callback
        rospy.Subscriber("sensor_trigger_front", Bool, self.sensor_front_cb)
        rospy.Subscriber("sensor_trigger_rear", Bool, self.sensor_rear_cb)
        rospy.Subscriber("sensor_kinect_color", Image, self.sensor_kinect_color_cb)
        rospy.Subscriber("sensor_kinect_depth", Float32MultiArray, self.sensor_kinect_depth_cb)
        rospy.Subscriber("sensor_arm_color", Image, self.sensor_arm_color_cb)
        rospy.Subscriber("sensor_arm_depth", Float32MultiArray, self.sensor_arm_depth_cb)
        rospy.Subscriber("sensor_front_color", Image, self.sensor_front_color_cb)
        rospy.Subscriber("sensor_front_depth", Float32MultiArray, self.sensor_front_depth_cb)
        rospy.Subscriber("sensor_UR3_joint_position_1", Float32, self.sensor_UR3_joint_position_cb_1)
        rospy.Subscriber("sensor_UR3_joint_position_2", Float32, self.sensor_UR3_joint_position_cb_2)
        rospy.Subscriber("sensor_UR3_joint_position_3", Float32, self.sensor_UR3_joint_position_cb_3)
        rospy.Subscriber("sensor_UR3_joint_position_4", Float32, self.sensor_UR3_joint_position_cb_4)
        rospy.Subscriber("sensor_UR3_joint_position_5", Float32, self.sensor_UR3_joint_position_cb_5)
        rospy.Subscriber("sensor_UR3_joint_position_6", Float32, self.sensor_UR3_joint_position_cb_6)
        rospy.Subscriber("sim_time", Float32, self.sim_time_cb)
        rospy.Subscriber("sensor_robot_position", Float32MultiArray, self.sensor_robot_position_cb)
        rospy.Subscriber("sensor_robot_angle", Float32MultiArray, self.sensor_robot_angle_cb)
        rospy.Subscriber("sensor_block_position", Float32MultiArray, self.sensor_block_position_cb)
        rospy.Subscriber("sensor_arm_blob", Float32MultiArray, self.sensor_arm_blob_cb)
        rospy.Subscriber("sensor_suction_state", Bool, self.sensor_suction_state_cb)



        #variables for storing data recieved from ROS subscriptions
        self.sensor_trigger_front = None
        self.sensor_trigger_rear = None
        self.sensor_kinect_color = None
        self.sensor_kinetic_depth = None
        self.sensor_arm_color = None
        self.sensor_arm_depth = None
        self.sensor_front_color = None
        self.sensor_front_depth = None
        self.sensor_UR3_joint_position_1 = 0.0
        self.sensor_UR3_joint_position_2 = 0.0
        self.sensor_UR3_joint_position_3 = 0.0
        self.sensor_UR3_joint_position_4 = 0.0
        self.sensor_UR3_joint_position_5 = 0.0
        self.sensor_UR3_joint_position_6 = 0.0
        self.sensor_UR3_joint_1_init = False
        self.sensor_UR3_joint_2_init = False
        self.sensor_UR3_joint_3_init = False
        self.sensor_UR3_joint_4_init = False
        self.sensor_UR3_joint_5_init = False
        self.sensor_UR3_joint_6_init = False
        self.robot_position = None
        self.robot_angle = None
        self.robot_position_init = False
        self.robot_angle_init = False
        self.block_position = None
        self.block_position_init = False
        self.sim_time = 0.0

        self.last_control_time = 0.0
        self.last_theta_e = 0.0

        self.sensor_arm_blob_coord = None
        self.sensor_suction_state = False
        
        self.arm_path = None
        self.arm_path_index = 0

        self.base_path = None
        self.base_path_index = 0


        self.block_found = False
        self.sensor_arm_blob_coord = None

        self.block_gripped = False
        self.block_released = False

        self.block_position_local = None

        #initialize csv files for logging
        self.front_sensor_log = 'front_sensor.csv' 
        if not path.exists(self.front_sensor_log):
            with open(self.front_sensor_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'Sensor Reading'])

        self.rear_sensor_log = 'rear_sensor.csv' 
        if not path.exists(self.rear_sensor_log):
            with open(self.rear_sensor_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'Sensor Reading'])

        #initialize csv files for logging
        self.to_block_theta_e_log = 'path_theta_e_toblock.csv' 
        if not path.exists(self.to_block_theta_e_log):
            with open(self.to_block_theta_e_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'theta_e', 'theta_e_dot', 'path_index'])

        self.to_block_position_log = 'path_position_toblock.csv' 
        if not path.exists(self.to_block_position_log):
            with open(self.to_block_position_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'position_x', 'position_y', 'prev_waypoint_x', 'prev_waypoint_y', 'current_waypoint_x', 'current_waypoint_y', 'path_index'])

        #initialize csv files for logging
        self.to_dest_theta_e_log = 'path_theta_e_todest.csv' 
        if not path.exists(self.to_dest_theta_e_log):
            with open(self.to_dest_theta_e_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'theta_e', 'theta_e_dot', 'path_index'])

        self.to_dest_position_log = 'path_position_todest.csv' 
        if not path.exists(self.to_dest_position_log):
            with open(self.to_dest_position_log, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Sim Time', 'position_x', 'position_y', 'prev_waypoint_x', 'prev_waypoint_y', 'current_waypoint_x', 'current_waypoint_y', 'path_index'])


        self.FK = forward_kinematics.ForwardKinematics()
        self.IK = inverse_kinematics.InverseKinematics()

    #continuously control the robot
    def run(self):
        
        #run until we die
        while not rospy.is_shutdown() and self.robot_state is not RobotState.FIN:

            #################################
            # INITIALIZATION MODULE BEGIN
            #################################
            if self.robot_state is RobotState.INITIALIZING:
                if (self.block_position_init and
                    self.robot_position_init and 
                    self.robot_angle_init and 
                    self.sensor_UR3_joint_1_init and 
                    self.sensor_UR3_joint_2_init and
                    self.sensor_UR3_joint_3_init and
                    self.sensor_UR3_joint_4_init and
                    self.sensor_UR3_joint_5_init and
                    self.sensor_UR3_joint_6_init):
                        if self.set_joints(0.0, 0.0, -np.pi/2, 0.0, 0.0, 0.0 ):
                            self.robot_state = RobotState.MOVE_BASE_TO_BLOCK
                            print('robot initialized, transition into MOVE_BASE_TO_BLOCK')

            #################################
            # INITIALIZATION MODULE END
            #################################


            #################################
            # TRAVEL TO BLOCK MODULE BEGIN
            #################################
            if self.robot_state is RobotState.MOVE_BASE_TO_BLOCK:
                if self.base_state is BaseState.STATIONARY:
                    current_position = np.array([[self.robot_position[0]],[self.robot_position[1]]])
                    intermediat_point = np.array([[-0.275],[0.475]])
                    block_pos = np.array([[self.block_position[0]],[self.block_position[1]]])
                    int_to_block_vec = block_pos - intermediat_point
                    int_to_block_norm = np.linalg.norm(int_to_block_vec)
                    path_end = intermediat_point + int_to_block_vec/int_to_block_norm*(int_to_block_norm-0.5)
                    
                    #create a path from our location to the block location
                    self.base_path_index = 1
                    path = []
                    path.append(current_position)
                    path.append(intermediat_point)
                    path.append(path_end)
                    path.append(intermediat_point)

                    self.base_path_global = path
                    self.base_state = BaseState.MOVING
                
                if self.base_state is BaseState.MOVING:
                    path_complete = self.move_base_path(self.base_path_global)        
                    
                    if path_complete:
                        self.base_state = BaseState.STATIONARY
                        self.robot_state = RobotState.MOVE_ARM_OVER_BLOCK
                        self.base_setpoint_global = None
                        print('MOVE_BASE_TO_BLOCK , transition into MOVE_ARM_OVER_BLOCK')

            #################################
            # TRAVEL TO BLOCK MODULE END
            #################################
            
            #################################
            # GRASP BLOCK MODULE BEGIN
            #################################
            if self.robot_state is RobotState.MOVE_ARM_OVER_BLOCK:
                if self.arm_state is ArmState.STATIONARY:
                    angle = 0
                    setpoint_local = np.array([[-0.5*np.cos(angle)],[0.5*np.sin(angle)],[.2]])
                    self.arm_path = self.get_arm_path(setpoint_local)
                    self.arm_path_index = 0
                    self.arm_state = ArmState.MOVING
                
                if self.arm_state is ArmState.MOVING:
                    path_complete = self.move_arm(self.arm_path)        
                
                    if path_complete:
                        self.arm_state = ArmState.STATIONARY
                        self.robot_state = RobotState.QUERYING_CAMERA
                        self.arm_path = None
                        self.arm_path_index = 0
                        print('MOVE_ARM_OVER_BLOCK , transition into QUERYING_CAMERA')

            if self.robot_state is RobotState.QUERYING_CAMERA:
                if not self.block_found:
                    print('Blob found at position (relative to end effector)', self.sensor_arm_blob_coord)         
                    self.block_found = True
                    S,M = self.FK.get_S_and_M()
                    theta = np.array([[self.sensor_UR3_joint_position_1],
                                      [self.sensor_UR3_joint_position_2],
                                      [self.sensor_UR3_joint_position_3],
                                      [self.sensor_UR3_joint_position_4],
                                      [self.sensor_UR3_joint_position_5],
                                      [self.sensor_UR3_joint_position_6]])
                    T = np.eye(4)
                    for i in range(0,len(theta)):
                        Si = S[:,i].reshape(-1,1)
                        Si_bracket = rotations.screw_to_bracket(Si)
                        thetai = theta[i]
                        T = np.matmul(T,expm(Si_bracket*thetai))

                    T = np.matmul(T,M)
                    suction_offset = 0.035
                    block_position_cam = np.array([[-self.sensor_arm_blob_coord[2]-suction_offset],
                                                   [-self.sensor_arm_blob_coord[1]],
                                                   [-self.sensor_arm_blob_coord[0]],
                                                   [1]])

                    self.block_position_local = np.matmul(T,block_position_cam)
                    print('block position (base frame)', self.block_position_local)
                    self.robot_state = RobotState.MOVING_ARM_TO_BLOCK
                    print('QUERYING_CAMERA , transition into MOVING_ARM_TO_BLOCK')

            if self.robot_state is RobotState.MOVING_ARM_TO_BLOCK:
                if self.arm_state is ArmState.STATIONARY:
                    setpoint_local = np.array([self.block_position_local[0],self.block_position_local[1],self.block_position_local[2]])
                    self.arm_path = self.get_arm_path(setpoint_local)
                    self.arm_path_index = 0
                    self.arm_state = ArmState.MOVING
                
                if self.arm_state is ArmState.MOVING:
                    path_complete = self.move_arm(self.arm_path)        
                
                    if path_complete:
                        self.arm_state = ArmState.STATIONARY
                        self.robot_state = RobotState.GRIPPING_BLOCK
                        self.arm_path = None
                        self.arm_path_index = 0
                        print('MOVING_ARM_TO_BLOCK , transition into GRIPPING_BLOCK')

            if self.robot_state is RobotState.GRIPPING_BLOCK:
                if not self.sensor_suction_state:
                    self.pub_suction.publish(True)
                    
                if self.sensor_suction_state:
                    self.robot_state = RobotState.MOVING_BLOCK_TO_STANDBY
                    print('GRIPPING_BLOCK , transition into MOVING_BLOCK_TO_STANDBY')

            if self.robot_state is RobotState.MOVING_BLOCK_TO_STANDBY:
                if self.arm_state is ArmState.STATIONARY:
                    self.arm_state = ArmState.MOVING
                
                if self.arm_state is ArmState.MOVING:
                    path_complete = self.set_joints(0.0,0.0,0.0,0.0,0.0,0.0)        

                    if path_complete:
                        self.arm_state = ArmState.STATIONARY
                        self.robot_state = RobotState.MOVING_BASE_TO_DESTINATION
                        self.arm_path = None
                        self.arm_path_index = 0
                        print('MOVING_BLOCK_TO_STANDBY , transition into MOVING_BASE_TO_DESTINATION')

            #################################
            # GRASP BLOCK MODULE END
            #################################

            #################################
            # TRAVEL TO DROPOFF MODULE BEGIN
            #################################
            if self.robot_state is RobotState.MOVING_BASE_TO_DESTINATION:
                if self.base_state is BaseState.STATIONARY:
                    self.base_path_index = 1
                    current_position = np.array([[self.robot_position[0]],[self.robot_position[1]]])
                    intermediat_point = np.array([[-1.0],[0.475]])
                    path_end = np.array([[-1.0],[0.9]])
                    path_end_facing = np.array([[0.0],[0.9]])

                    path = []
                    path.append(current_position)
                    path.append(intermediat_point)
                    path.append(path_end)
                    path.append(path_end_facing)

                    self.base_path_global = path        
                    self.base_state = BaseState.MOVING
                
                if self.base_state is BaseState.MOVING:
                    path_complete = self.move_base_path(self.base_path_global)        
                
                    if path_complete:
                        self.base_state = BaseState.STATIONARY
                        self.robot_state = RobotState.MOVING_BLOCK_TO_DESTINATION
                        self.base_setpoint_global = None
                        print('MOVING_BASE_TO_DESTINATION , transition into MOVING_BLOCK_TO_DESTINATION')

            if self.robot_state is RobotState.MOVING_BLOCK_TO_DESTINATION:
                if self.arm_state is ArmState.STATIONARY:
                    self.arm_state = ArmState.MOVING
                
                if self.arm_state is ArmState.MOVING:
                    path_complete = self.set_joints(0.0,0.0,0.0,0.0,-np.pi/2,0.0)        
                
                    if path_complete:
                        self.arm_state = ArmState.STATIONARY
                        self.robot_state = RobotState.RELEASING_BLOCK
                        print('MOVING_BLOCK_TO_DESTINATION , transition into RELEASING_BLOCK')

            #################################
            # TRAVEL TO DROPOFF MODULE END
            #################################

            #################################
            # RELEASE BLOCK MODULE BEGIN
            #################################
            if self.robot_state is RobotState.RELEASING_BLOCK:
                if self.sensor_suction_state:
                    self.pub_suction.publish(False)
                    
                if not self.sensor_suction_state:
                    self.robot_state = RobotState.FIN
                    print('RELEASING_BLOCK , transition into FIN')

            #################################
            # RELEASE BLOCK MODULE BEGIN
            #################################
    
            self.rate.sleep()

        #task complete
        print('el fin')


    #move the base through a series of waypoints, the last waypoint in the path is the point that the robot should be facing
    #once the rest of the path is complete (it doesnt actually travel to the last point in the input path)
    def move_base_path(self, path):
        
        path_length = len(path)
        
        if self.base_path_index < path_length-1:
            if self.move_base(path[self.base_path_index]):
                self.base_path_index = self.base_path_index + 1

            return False
        elif self.base_path_index == path_length-1:
            if self.move_base_align(path[self.base_path_index]):
                self.base_path_index = self.base_path_index + 1
            
            return False
        else:
            self.base_path_index = 0
            self.base_path_global = None
            return True
        

    #keep the base stationary while aligning its front with the provided position
    def move_base_align(self, position): 
        #parameters for pid control of the base
        Kp = .25
        Kd = 0.5

        waypointX = position[0]
        waypointY = position[1]
        Vc = 0.0
        rR = 0.1
        rL = 0.1

        robotDirectionX = np.cos(self.robot_angle[2])
        robotDirectionY = np.sin(self.robot_angle[2])
        robotDirectionMag = 1
        
        waypointDirectionX = waypointX - self.robot_position[0]
        waypointDirectionY = waypointY - self.robot_position[1]
        waypointDirectionMag = np.sqrt(waypointDirectionX*waypointDirectionX + waypointDirectionY*waypointDirectionY) 
    
        theta_e = np.arccos((robotDirectionX*waypointDirectionX + robotDirectionY*waypointDirectionY)/(robotDirectionMag*waypointDirectionMag))
        cp = robotDirectionX*waypointDirectionY - robotDirectionY*waypointDirectionX
        
        if (cp < 0):
            theta_e = theta_e*-1
        else:
            theta_e = theta_e

        current_control_time = rospy.Time.now().to_sec()
        dt = current_control_time - self.last_control_time
        theta_e_dot = (theta_e - self.last_theta_e)/dt

        motorSpeedLeft  = 0
        motorSpeedRight = 0
        if(dt is not 0.0):
            if(theta_e is not 0.0):
                motorSpeedLeft =  1/rL*(Vc - Kp*theta_e - Kd*theta_e_dot)
                motorSpeedRight = 1/rR*(Vc + Kp*theta_e + Kd*theta_e_dot)
            else:
                motorSpeedLeft =  1/rL*Vc
                motorSpeedRight = 1/rR*Vc
            
            self.pub_lms.publish(motorSpeedLeft)
            self.pub_rms.publish(motorSpeedRight)
        
        self.last_control_time = current_control_time
        self.last_theta_e = theta_e

        if np.abs(theta_e) < 0.05:
            
            self.pub_lms.publish(0.0)
            self.pub_rms.publish(0.0)
            return True
        else:
            return False

    #move the base to the provided position
    def move_base(self, position): 
        #parameters for pid control of the base
        Kp = .25
        Kd = .5
        
        waypointX = position[0]
        waypointY = position[1]
        Vc = 0.3
        rR = 0.1
        rL = 0.1

        robotDirectionX = np.cos(self.robot_angle[2])
        robotDirectionY = np.sin(self.robot_angle[2])
        robotDirectionMag = 1

        waypointDirectionX = waypointX - self.robot_position[0]
        waypointDirectionY = waypointY - self.robot_position[1]
        waypointDirectionMag = np.sqrt(waypointDirectionX*waypointDirectionX + waypointDirectionY*waypointDirectionY) 
        
        theta_e = np.arccos((robotDirectionX*waypointDirectionX + robotDirectionY*waypointDirectionY)/(robotDirectionMag*waypointDirectionMag))
        cp = robotDirectionX*waypointDirectionY - robotDirectionY*waypointDirectionX
                
        if (cp < 0):
            theta_e = theta_e*-1
        else:
            theta_e = theta_e
    
        current_control_time = rospy.Time.now().to_sec()
        dt = current_control_time - self.last_control_time
        theta_e_dot = (theta_e - self.last_theta_e)/dt

        if waypointDirectionMag < 0.2:
            Vc = 0.1

        if np.abs(theta_e) > np.pi/6 or np.abs(theta_e_dot) > 0.1:
            Vc = 0.0

        motorSpeedLeft  = 0
        motorSpeedRight = 0
        if(dt is not 0.0):
            if(theta_e is not 0.0):
                motorSpeedLeft =  1/rL*(Vc - Kp*theta_e - Kd*theta_e_dot)
                motorSpeedRight = 1/rR*(Vc + Kp*theta_e + Kd*theta_e_dot)
            else:
                motorSpeedLeft =  1/rL*Vc
                motorSpeedRight = 1/rR*Vc
            
            self.pub_lms.publish(motorSpeedLeft)
            self.pub_rms.publish(motorSpeedRight)
            
        self.last_control_time = current_control_time
        self.last_theta_e = theta_e

        position_log = self.to_dest_position_log
        theta_log = self.to_dest_theta_e_log
        if self.robot_state is RobotState.MOVE_BASE_TO_BLOCK:
            position_log = self.to_block_position_log
            theta_log = self.to_block_theta_e_log

        #log the sensor readings
        with open(theta_log, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.sim_time, theta_e[0], theta_e_dot[0], self.base_path_index])
        
        wp_prev = self.base_path_global[self.base_path_index-1]
        #log the sensor readings
        with open(position_log, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.sim_time, self.robot_position[0], self.robot_position[1], wp_prev[0][0], wp_prev[1][0], waypointX[0], waypointY[0], self.base_path_index])

        if np.abs(waypointDirectionMag) <  0.05:
            
            self.pub_lms.publish(0.0)
            self.pub_rms.publish(0.0)
            return True
        else:
            return False

    #set the target joint angles for the UR3
    def set_joints(self,theta1,theta2,theta3,theta4,theta5,theta6): 
        
        self.pub_UR3_joint_target_1.publish(theta1)
        self.pub_UR3_joint_target_2.publish(theta2)
        self.pub_UR3_joint_target_3.publish(theta3)
        self.pub_UR3_joint_target_4.publish(theta4)
        self.pub_UR3_joint_target_5.publish(theta5)
        self.pub_UR3_joint_target_6.publish(theta6)

        if (np.abs(self.sensor_UR3_joint_position_1-theta1) < 0.05 and
            np.abs(self.sensor_UR3_joint_position_2-theta2) < 0.05 and
            np.abs(self.sensor_UR3_joint_position_3-theta3) < 0.05 and
            np.abs(self.sensor_UR3_joint_position_4-theta4) < 0.05 and
            np.abs(self.sensor_UR3_joint_position_5-theta5) < 0.05 and
            np.abs(self.sensor_UR3_joint_position_6-theta6) < 0.05):
        
            return True
        else:
            return False

    #get a path of joint angles that will smoothly move the end effector to the provided position
    def get_arm_path(self, block_position_local):
        #first get the path
        R = np.array([[0.0,0.0,1.0],[0.0,-1.0,0.0],[1.0,0.0,0.0]])
        p = block_position_local
        
        T_desired = np.hstack([R,p])
        T_desired = np.vstack([T_desired,[0.0, 0.0, 0.0, 1.0]])

        thetas0 = np.zeros((1,6))
        thetas0[0,0] = self.sensor_UR3_joint_position_1+.00001
        thetas0[0,1] = self.sensor_UR3_joint_position_2+.00001
        thetas0[0,2] = self.sensor_UR3_joint_position_3+.00001
        thetas0[0,3] = self.sensor_UR3_joint_position_4+.00001
        thetas0[0,4] = self.sensor_UR3_joint_position_5+.00001
        thetas0[0,5] = self.sensor_UR3_joint_position_6+.00001

        #do inverse kinematics to find joint angles
        S,M = self.FK.get_S_and_M()
        return self.IK.get_thetas_path(T_desired, M, S, thetas0)


    #returns false while path is in progress and true when path is complete 
    def move_arm(self, arm_path):
        if self.arm_path_index < arm_path.shape[0]:    
            thetas_path_i = arm_path[self.arm_path_index,:]

            self.pub_UR3_joint_target_1.publish(thetas_path_i[0])
            self.pub_UR3_joint_target_2.publish(thetas_path_i[1])
            self.pub_UR3_joint_target_3.publish(thetas_path_i[2])
            self.pub_UR3_joint_target_4.publish(thetas_path_i[3])
            self.pub_UR3_joint_target_5.publish(thetas_path_i[4])
            self.pub_UR3_joint_target_6.publish(thetas_path_i[5])
        
            self.arm_path_index = self.arm_path_index + 1
            return False
        else:
            return True

    #callback to handle data received from the sensor_trigger_rear topic
    def sensor_front_cb(self,data):
        self.sensor_trigger_front = data.data

        #if sense an object, go in reverse
        if data.data == True:
            self.robot_direction = -1.0

        #log the sensor readings
        with open(self.front_sensor_log, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.sim_time, data.data])

    #callback to handle data received from the sensor_trigger_rear topic
    def sensor_rear_cb(self,data):
        self.sensor_trigger_rear = data.data
        
        #if sense an object, go forward
        if data.data == True:
            self.robot_direction = 1.0

        #log the sensor readings
        with open(self.rear_sensor_log, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.sim_time, data.data])

    #callback to handle data received from the sensor_kinetic_depth topic
    def sensor_kinect_depth_cb(self,data):
        self.sensor_kinect_depth = data.data

    #callback to handle data received from the sensor_kinetic_color topic
    def sensor_kinect_color_cb(self,data):
        self.sensor_kinect_color = data.data
    
    #callback to handle data received from the sensor_arm_depth topic
    def sensor_arm_depth_cb(self,data):
        self.sensor_arm_depth = data.data

    #callback to handle data received from the sensor_arm_color topic
    def sensor_arm_color_cb(self,data):
        self.sensor_arm_color = data.data

    #callback to handle data received from the sensor_front_depth topic
    def sensor_front_depth_cb(self,data):
        self.sensor_front_depth = data.data

    #callback to handle data received from the sensor_front_color topic
    def sensor_front_color_cb(self,data):
        self.sensor_front_color = data.data

    #callback to handle data received from the sensor_UR3_joint_position_1 topic
    def sensor_UR3_joint_position_cb_1(self,data):
        self.sensor_UR3_joint_position_1 = data.data
        self.sensor_UR3_joint_1_init = True

    #callback to handle data received from the sensor_UR3_joint_position_2 topic
    def sensor_UR3_joint_position_cb_2(self,data):
        self.sensor_UR3_joint_position_2 = data.data
        self.sensor_UR3_joint_2_init = True

    #callback to handle data received from the sensor_UR3_joint_position_3 topic
    def sensor_UR3_joint_position_cb_3(self,data):
        self.sensor_UR3_joint_position_3 = data.data
        self.sensor_UR3_joint_3_init = True

    #callback to handle data received from the sensor_UR3_joint_position_4 topic
    def sensor_UR3_joint_position_cb_4(self,data):
        self.sensor_UR3_joint_position_4 = data.data
        self.sensor_UR3_joint_4_init = True

    #callback to handle data received from the sensor_UR3_joint_position_5 topic
    def sensor_UR3_joint_position_cb_5(self,data):
        self.sensor_UR3_joint_position_5 = data.data
        self.sensor_UR3_joint_5_init = True

    #callback to handle data received from the sensor_UR3_joint_position_6 topic
    def sensor_UR3_joint_position_cb_6(self,data):
        self.sensor_UR3_joint_position_6 = data.data
        self.sensor_UR3_joint_6_init = True

    #callback to handle data received from the simTime topic
    def sim_time_cb(self,data):
        self.sim_time = data.data

    def sensor_arm_blob_cb(self,data):
        self.sensor_arm_blob_coord = data.data
        
    def sensor_suction_state_cb(self,data):
        self.sensor_suction_state = data.data
        
    def sensor_robot_position_cb(self,data):
        self.robot_position = data.data
        self.robot_position_init = True
        
    def sensor_robot_angle_cb(self,data):
        self.robot_angle = data.data
        self.robot_angle_init = True
        
    def sensor_block_position_cb(self,data):
        self.block_position = data.data
        self.block_position_init = True
        
    
if __name__ == '__main__':
    try:
        #initialize the controller class
        controller = RobotController()
        #run the controller class
        controller.run()
    except rospy.ROSInterruptException:
        pass

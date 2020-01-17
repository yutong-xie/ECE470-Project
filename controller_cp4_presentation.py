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

import os.path
from os import path
import csv


class RobotController:

    #initialize class instance
    def __init__(self):    
    
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

        #added
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

        #added
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
        self.sim_time = 0.0

        #added
        self.sensor_arm_blob_coord = None
        self.sensor_suction_state = False

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

        self.FK = forward_kinematics.ForwardKinematics()
        self.IK = inverse_kinematics.InverseKinematics()

    #continuously control the robot
    def run(self):

        # set desired transform of end-effector
        R = np.array([[0.0,0.0,1.0],[0.0,-1.0,0.0],[1.0,0.0,0.0]])

        p = np.array([[0.4],[-0.35],[0.1]])
        
        
        T_desired = np.hstack([R,p])
        T_desired = np.vstack([T_desired,[0.0, 0.0, 0.0, 1.0]])

        print('The desired transformation is:',T_desired)

        #do inverse kinematics to find joint angles
        S,M = self.FK.get_S_and_M()
        thetas = self.IK.get_thetas(T_desired, M, S)

        #publish joint angles

        # self.pub_UR3_joint_target_1.publish(0.0)
        # self.pub_UR3_joint_target_2.publish(0.0)
        # self.pub_UR3_joint_target_3.publish(0.0)
        # self.pub_UR3_joint_target_4.publish(0.0)
        # self.pub_UR3_joint_target_5.publish(0.0)
        # self.pub_UR3_joint_target_6.publish(0.0)



        # # # joint_target = 3.14/10.0*np.sin(self.sim_time)
        self.pub_UR3_joint_target_1.publish(thetas[0])
        self.pub_UR3_joint_target_2.publish(thetas[1])
        self.pub_UR3_joint_target_3.publish(thetas[2])
        self.pub_UR3_joint_target_4.publish(thetas[3])
        self.pub_UR3_joint_target_5.publish(thetas[4])
        self.pub_UR3_joint_target_6.publish(thetas[5])
        

        #run until we die
        while not rospy.is_shutdown():
             self.pub_suction.publish(False)
             self.rate.sleep()

            
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

    #callback to handle data received from the sensor_UR3_joint_position_2 topic
    def sensor_UR3_joint_position_cb_2(self,data):
        self.sensor_UR3_joint_position_2 = data.data

    #callback to handle data received from the sensor_UR3_joint_position_3 topic
    def sensor_UR3_joint_position_cb_3(self,data):
        self.sensor_UR3_joint_position_3 = data.data

    #callback to handle data received from the sensor_UR3_joint_position_4 topic
    def sensor_UR3_joint_position_cb_4(self,data):
        self.sensor_UR3_joint_position_4 = data.data

    #callback to handle data received from the sensor_UR3_joint_position_5 topic
    def sensor_UR3_joint_position_cb_5(self,data):
        self.sensor_UR3_joint_position_5 = data.data

    #callback to handle data received from the sensor_UR3_joint_position_6 topic
    def sensor_UR3_joint_position_cb_6(self,data):
        self.sensor_UR3_joint_position_6 = data.data

    #callback to handle data received from the simTime topic
    def sim_time_cb(self,data):
        self.sim_time = data.data

    #added
    def sensor_arm_blob_cb(self,data):
        self.sensor_arm_blob_coord = data.data
        print("blob center x,y,z: ", self.sensor_arm_blob_coord)

    def sensor_suction_state_cb(self,data):
        print ("suction cb called")
        self.sensor_suction_state = data.data
        print("is it gripped: ", self.sensor_suction_state)



    
if __name__ == '__main__':
    try:
        #initialize the controller class
        controller = RobotController()
        #run the controller class
        controller.run()
    except rospy.ROSInterruptException:
        pass

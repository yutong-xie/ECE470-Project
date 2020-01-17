#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
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
    
        #variables for storing data recieved from ROS subscriptions
        self.sensor_trigger_front = None
        self.sensor_trigger_rear = None
        self.sensor_kinect_color = None
        self.sensor_kinetic_depth = None
        self.sensor_arm_color = None
        self.sensor_arm_depth = None
        self.sensor_front_color = None
        self.sensor_front_depth = None
        self.sensor_UR3_joint_position_1 = None
        self.sensor_UR3_joint_position_2 = None
        self.sensor_UR3_joint_position_3 = None
        self.sensor_UR3_joint_position_4 = None
        self.sensor_UR3_joint_position_5 = None
        self.sensor_UR3_joint_position_6 = None
        self.sim_time = 0.0

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


    #continuously control the robot
    def run(self):

        #run until we die
        while not rospy.is_shutdown():

            # rospy.loginfo(self.robot_direction*self.robot_motor_speed) #print to screen, write to node logfile, and write to rosout
            
            self.pub_rms.publish(self.robot_direction*self.robot_motor_speed) #publish to the topic that controls the right motor
            self.pub_lms.publish(self.robot_direction*self.robot_motor_speed) #publish to the topic that controls the left motor

            joint_target = 3.14/10.0*np.sin(self.sim_time)
            self.pub_UR3_joint_target_1.publish(joint_target)
            self.pub_UR3_joint_target_2.publish(joint_target)
            self.pub_UR3_joint_target_3.publish(joint_target)
            self.pub_UR3_joint_target_4.publish(joint_target)
            self.pub_UR3_joint_target_5.publish(joint_target)
            self.pub_UR3_joint_target_6.publish(joint_target)


            #pause loop based on execution frequency
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


    
if __name__ == '__main__':
    try:
        #initialize the controller class
        controller = RobotController()
        #run the controller class
        controller.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import CompressedImage, Image
import time
from __builtin__ import True
#bridge = CvBridge()
class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.pic = None
        self.count = 0
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()
        #decide which robot
        self.robot = "robot"
        self.robot1 = "robot1"
        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)
        self.pub_pressA = rospy.Publisher("~press_A",BoolStamped,queue_size=1)
        self.pub_pressB = rospy.Publisher("~press_B",BoolStamped,queue_size=1)
        self.pub_pressX = rospy.Publisher("~press_X",BoolStamped,queue_size=1)
        self.pub_pressY = rospy.Publisher("~press_Y",BoolStamped,queue_size=1)
        self.save_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
#        self.pub_picture = rospy.Subscriber("~photo",sensor_msgs,queue_size=1))
        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)
    
    def MultiRobot(self):
        self.pub_car_cmd = rospy.Publisher("/"+self.robot+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_pressA = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_A",BoolStamped,queue_size=1)
        self.pub_pressB = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_B",BoolStamped,queue_size=1)
        self.pub_pressX = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_X",BoolStamped,queue_size=1)
        self.pub_pressY = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_Y",BoolStamped,queue_size=1) 
    def MultiRobot1(self):
        self.pub_car_cmd = rospy.Publisher("/"+self.robot1+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_pressA = rospy.Publisher("/"+self.robot1+"/joy_mapper_node/press_A",BoolStamped,queue_size=1)
        self.pub_pressB = rospy.Publisher("/"+self.robot1+"/joy_mapper_node/press_B",BoolStamped,queue_size=1)
        self.pub_pressX = rospy.Publisher("/"+self.robot1+"/joy_mapper_node/press_X",BoolStamped,queue_size=1)
        self.pub_pressY = rospy.Publisher("/"+self.robot1+"/joy_mapper_node/press_Y",BoolStamped,queue_size=1)

    def cbImage(self,msg):
        image_cv = image_cv_from_jpg(msg.data)
        self.pic = image_cv

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
        self.MultiRobot()
        self.pub_car_cmd.publish(car_cmd_msg)
        self.MultiRobot1()
        self.pub_car_cmd.publish(car_cmd_msg)

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10

    def processButtons(self, joy_msg):
        if (joy_msg.buttons[6] == 1): #The back button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            self.pub_joy_override.publish(override_msg)
        elif (joy_msg.buttons[7] == 1): #the start button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            self.pub_joy_override.publish(override_msg)
        elif (joy_msg.buttons[5] == 1): # Right back button
            self.state_verbose ^= True
            rospy.loginfo('state_verbose = %s' % self.state_verbose)
            rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param

        elif (joy_msg.buttons[4] == 1): #Left back button
            self.state_parallel_autonomy ^= True
            rospy.loginfo('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            parallel_autonomy_msg = BoolStamped()
            parallel_autonomy_msg.header.stamp = self.joy.header.stamp
            parallel_autonomy_msg.data = self.state_parallel_autonomy
            self.pub_parallel_autonomy.publish(parallel_autonomy_msg)
            """
        elif (joy_msg.buttons[3] == 1):
            pressY_msg = BoolStamped()
            rospy.loginfo('Press "Y"')
            rospy.loginfo('HELLO WORLD~~~~~~~')
            pressY_msg.header.stamp = self.joy.header.stamp
            pressY_msg.data = True 
            self.pub_pressY.publish(pressY_msg)
        elif (joy_msg.buttons[2] == 1):
            pressX_msg = BoolStamped()
            #raspistill -t 1000 -o out1.jpg
            rospy.loginfo('Press "X"')
            rospy.loginfo('Let us take a picture~~~~~~~')
            cv2.imwrite("/home/ubuntu/duckietown/demo"+".jpg",self.pic)
            #cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            pressX_msg.header.stamp = self.joy.header.stamp
            pressX_msg.data = True 
            self.pub_pressX.publish(pressX_msg)
            """
        elif (joy_msg.buttons[3] == 1):
            closeled_msg = BoolStamped()
            closeled_msg.header.stamp = self.joy.header.stamp
            rospy.loginfo('Press "Y"')
            self.count += 1
            if (self.count >= 3):
                self.count = 0
            if (self.count == 0):
                self.robot = "arg1"
                self.robot1 = "robot1"
                closeled_msg.data = False 
                self.pub_pressY = rospy.Publisher("/arg2/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                closeled_msg.data = True 
                self.pub_pressY = rospy.Publisher("/arg1/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                rospy.loginfo('Select "arg1"')
            if (self.count == 1):
                self.robot = "arg2"
                self.robot1 = "robot1"
                closeled_msg.data = False 
                self.pub_pressY = rospy.Publisher("/arg1/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                closeled_msg.data = True 
                self.pub_pressY = rospy.Publisher("/arg2/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                rospy.loginfo('Select "arg2"')
            if (self.count == 2):
                self.robot = "arg1"
                self.robot1 = "arg2"
                closeled_msg.data = True 
                self.pub_pressY = rospy.Publisher("/arg2/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                closeled_msg.data = True 
                self.pub_pressY = rospy.Publisher("/arg1/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
                self.pub_pressY.publish(closeled_msg)
                rospy.loginfo('Select "all car"')
        elif (joy_msg.buttons[2] == 1):
            pressX_msg = BoolStamped()
            #raspistill -t 1000 -o out1.jpg
            rospy.loginfo('Press "X"')
            rospy.loginfo('Select "arg1"')
            #self.robot = "car13"
            #self.MultiRobot()
            #pressX_msg.header.stamp = self.joy.header.stamp
            #pressX_msg.data = True 
            #self.pub_pressX.publish(pressX_msg)
        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            self.pub_e_stop.publish(e_stop_msg)
        elif (joy_msg.buttons[9] == 1): #push left joystick button 
            avoidance_msg = BoolStamped()
            rospy.loginfo('start lane following with avoidance mode')
            avoidance_msg.header.stamp = self.joy.header.stamp
            avoidance_msg.data = True 
            self.pub_avoidance.publish(avoidance_msg)
        elif (joy_msg.buttons[0] == 1): #push left joystick button 
            pressA_msg = BoolStamped()
            rospy.loginfo('Press "A"')
            rospy.loginfo('Joystick Control')
            pressA_msg.header.stamp = self.joy.header.stamp
            pressA_msg.data = True 
            self.MultiRobot()
            self.pub_pressA.publish(pressA_msg)
            self.MultiRobot1()
            self.pub_pressA.publish(pressA_msg)
        elif (joy_msg.buttons[1] == 1): #push left joystick button 
            pressB_msg = BoolStamped()
            rospy.loginfo('Press "B"')
            rospy.loginfo('Lane following')
            pressB_msg.header.stamp = self.joy.header.stamp
            pressB_msg.data = True 
            self.MultiRobot()
            self.pub_pressB.publish(pressB_msg)
            self.MultiRobot1()
            self.pub_pressB.publish(pressB_msg)
        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))
                                          

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()

#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from patrolling_msgs.msg import RobotName
from geometry_msgs.msg import Twist
import sys
import math
from sensor_msgs.msg import Joy
import time
from __builtin__ import True

class carName(object):
    def __init__(self):
        #self.node_name = rospy.get_name()
        #decide which robot
        self.robot = "robot"
        self.robotlist = {}
        self.rlist = []
        self.rnumber = 2
        self.rcount = 1
        self.allrb = False
        # Setup parameters
        self.name = "master"
        # Publicaitons
        self.pub_name = rospy.Publisher("/stop_sending", RobotName, queue_size=1)
        #Subscriptions
        self.sub_name = rospy.Subscriber('/robot_name', RobotName, self.subname, queue_size=1)

        self.pic = None
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()
        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_twist = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
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

    def count (self):
        r = rospy.Rate(1)
        for i in self.robotlist.keys():
            self.robotlist[i] = self.robotlist[i] + 1
            if self.robotlist[i] >=10 :
                self.robotlist.pop(i)
                self.rlist.remove(i)

    def subname(self, msg):
        if msg.send == True :
            if msg.robot_name not in self.robotlist :
                self.robotlist[msg.robot_name]=0
                self.rlist.append(msg.robot_name)
            else :
                self.robotlist[msg.robot_name]=0
        #self.rnumber = len(self.rlist)
        self.count()

    def AllRobot(self, pub, msg):
        for i in self.robotlist.keys():
            self.robot = i
            self.pub_car_cmd = rospy.Publisher("/"+self.robot+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
            self.pub_pressA = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_A",BoolStamped,queue_size=1)
            self.pub_pressB = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_B",BoolStamped,queue_size=1)
            self.pub_pressX = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_X",BoolStamped,queue_size=1)
            self.pub_pressY = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_Y",BoolStamped,queue_size=1)
            if pub == 1:
                self.pub_car_cmd.publish(msg)
            elif pub == 2:
                self.pub_pressA.publish(msg)
            elif pub == 3:
                self.pub_pressB.publish(msg)
            elif pub == 4:
                self.pub_pressX.publish(msg)
            elif pub == 5:
                self.pub_pressY.publish(msg)
            #rospy.sleep(0.1)

    def MultiRobot(self):
        self.pub_car_cmd = rospy.Publisher("/"+self.robot+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_pressA = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_A",BoolStamped,queue_size=1)
        self.pub_pressB = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_B",BoolStamped,queue_size=1)
        self.pub_pressX = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_X",BoolStamped,queue_size=1)
        self.pub_pressY = rospy.Publisher("/"+self.robot+"/joy_mapper_node/press_Y",BoolStamped,queue_size=1) 

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
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
        car_twist_msg = Twist()
        car_twist_msg.linear.x = car_cmd_msg.v*1.5
        car_twist_msg.angular.z = car_cmd_msg.omega*1.5
        self.pub_car_cmd.publish(car_cmd_msg)
        
        self.pub_car_twist.publish(car_twist_msg)


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
        elif (joy_msg.buttons[3] == 1):
            rospy.loginfo('Press "Y"')
            if self.rcount == 1:
                self.robot = "lakers"
                print "choose", self.robot
            elif self.rcount == 2:
                self.robot = "celtics"
                print "choose", self.robot
            elif self.rcount > 2:
                self.rcount = 1
                self.robot = "lakers"
                print "choose", self.robot
            self.rcount += 1
            self.pub_car_twist = rospy.Publisher("/"+self.robot+"/cmd_vel",Twist,queue_size=1)
            #self.pub_car_twist = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

        elif (joy_msg.buttons[2] == 1):
            print "--------- ", len(self.robotlist), " ---------"
            print self.rlist
            print
            print
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
        elif (joy_msg.buttons[0] == 1): #press A 
            pressA_msg = BoolStamped()
            rospy.loginfo('Press "A"')
            rospy.loginfo('Joystick Control')
            pressA_msg.header.stamp = self.joy.header.stamp
            pressA_msg.data = True 
            if self.allrb == True:
                self.AllRobot(2,pressA_msg)
            else:
                #self.MultiRobot()
                self.pub_pressA.publish(pressA_msg)
        elif (joy_msg.buttons[1] == 1): #press B
            pressB_msg = BoolStamped()
            rospy.loginfo('Press "B"')
            rospy.loginfo('Lane following')
            pressB_msg.header.stamp = self.joy.header.stamp
            pressB_msg.data = True 
            if self.allrb == True:
                self.AllRobot(3,pressB_msg)
            else:
                #self.MultiRobot()
                self.pub_pressB.publish(pressB_msg)
        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))

if __name__ == "__main__":
    rospy.init_node("robot_name",anonymous=False)
    name_node = carName()
    rospy.spin()

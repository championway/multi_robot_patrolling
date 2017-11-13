#!/usr/bin/env python
import rospy
import numpy as np
from patrolling_msgs.msg import RobotName
import sys
        
class carName(object):
    def __init__(self):
        #self.node_name = rospy.get_name()

        # Setup parameters
        self.stop = False
        self.name = rospy.get_param('~name')
        # Publicaiton
        self.pub_name = rospy.Publisher("/robot_name", RobotName, queue_size=1)

        # safe shutdown
        #self.sub_name = rospy.Subscriber("/"+self.name+"/stop_sending", RobotName, self.subname, queue_size=1)

        self.publishName()

    def publishName(self):
        rospy.loginfo("Test Test Test")
        name_msg = RobotName()
        name_msg.send = True
        name_msg.robot_name = self.name
        r = rospy.Rate(2)
        #while self.stop == False :
        while True :
            self.pub_name.publish(name_msg)
            r.sleep()

    '''def subname(self, msg):
        self.stop = msg.send
        rospy.loginfo(msg)'''

if __name__ == "__main__":
    rospy.init_node("robot_name",anonymous=False)
    name_node = carName()
    rospy.spin()

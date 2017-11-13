#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, BoolStamped
import time
from geometry_msgs.msg import Twist

class CarCmdSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        # rospy.loginfo("[%s] Initializing " %(self.node_name))
        # Read parameters
        self.mappings = rospy.get_param("~mappings")
        source_topic_dict = rospy.get_param("~source_topics")
        self.current_src_name = None
        self.state = None
        # Construct publisher
        self.pub_cmd = rospy.Publisher("~cmd",Twist2DStamped,queue_size=1)
        self.pub_stop_around = rospy.Publisher("~stop_around", BoolStamped, queue_size=1, latch=True)
        # Construct subscribers
        self.pub_car_twist = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.sub_fsm_state = rospy.Subscriber(rospy.get_param("~mode_topic"),FSMState,self.cbFSMState)
        self.sub_dict = dict()
        for src_name, topic_name in source_topic_dict.items():
            self.sub_dict[src_name] = rospy.Subscriber(topic_name,Twist2DStamped,self.cbWheelsCmd,callback_args=src_name)

        rospy.loginfo("[%s] Initialized. " %(self.node_name))
    def cbFSMState(self,fsm_state_msg):
        self.current_src_name = self.mappings.get(fsm_state_msg.state)
        if self.current_src_name == "stop":
            print "wait for stop"
            self.pubStop()
        elif self.current_src_name == "around":
            print "start to turn around"
            self.pubTurn()
        elif self.current_src_name is None:
            rospy.logwarn("[%s] FSMState %s not handled. No msg pass through the switch." %(self.node_name,fsm_state_msg.state))
        else: 
            rospy.loginfo("[%s] Car cmd switched to %s in state %s." %(self.node_name,self.current_src_name,fsm_state_msg.state))

    def cbWheelsCmd(self,msg,src_name):
        if src_name == self.current_src_name:
            self.pub_cmd.publish(msg)
            car_twist_msg = Twist()
            car_twist_msg.linear.x = msg.v*1.5
            car_twist_msg.angular.z = msg.omega*3
            self.pub_car_twist.publish(car_twist_msg)

    def pubStop(self):
        msg = Twist2DStamped()
        msg.v = 0
        msg.omega = 0
        self.pub_cmd.publish(msg)
        car_twist_msg = Twist()
        car_twist_msg.linear.x = msg.v*1.5
        car_twist_msg.angular.z = msg.omega*3
        self.pub_car_twist.publish(car_twist_msg)

    def pubTurn(self):
        msg = Twist2DStamped()
        msg.v = 0
        msg.omega = 4
        self.pub_cmd.publish(msg)
        car_twist_msg = Twist()
        car_twist_msg.linear.x = msg.v*1.5
        car_twist_msg.angular.z = msg.omega*3
        self.pub_car_twist.publish(car_twist_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('car_cmd_switch_node', anonymous=False)
    # Create the DaguCar object
    node = CarCmdSwitchNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

#! /usr/bin/env python


import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class ActionBridge(object):
    
    def __init__(self):
        self._name = 'bridge_action'


    def init_vel(self):
        self._pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)


    def set_vel(self, linear_x, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self._pub_vel.publish(vel_msg)


    def callback(self, msg):
        action = msg.data
        linear_x = action[0]
        angular_z = action[1]
        self.set_vel(linear_x, angular_z)


    def run(self, hz=100):
        rospy.init_node(self._name)
        self.init_vel()
        
        rate = rospy.Rate(hz)

        sub = rospy.Subscriber('/'+self._name, Float64MultiArray, self.callback)
        
        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    ActionBridge().run()

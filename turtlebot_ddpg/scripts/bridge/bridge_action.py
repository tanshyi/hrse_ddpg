#! /usr/bin/env python


import time
import json
from datetime import datetime

import rospy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class ActionBridge(object):
    
    def __init__(self):
        self._name = 'bridge_action'


    def init_gazebo(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        self._svc_reset = rospy.ServiceProxy('gazebo/reset_simulation', Empty)

        rospy.wait_for_service('/gazebo/set_model_state')
        self._svc_set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


    def init_vel(self):
        self._pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)


    def set_vel(self, linear_x, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self._pub_vel.publish(vel_msg)


    def callback(self, msg):
        action = msg.data
        linear_x = action[1]
        angular_z = action[0]
        self.set_vel(linear_x, angular_z)


    def command_callback(self, msg):
        cmd = json.loads(msg.data)
        if cmd['command'] == 'reset':
            self.reset_world(target=(cmd['target_x'], cmd['target_y']))

    
    def reset_world(self, target):
        print('reset world: ' + str(datetime.now()))
        self.set_vel(0.0, 0.0)
        time.sleep(0.1)
        self.set_vel(0.0, 0.0)

        try:
            state_msg = ModelState()    
            state_msg.model_name = 'turtlebot3_waffle_pi'
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 0.0
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 1.0
            self._svc_set_state(state_msg)
        except rospy.ServiceException as e:
            print("reset model failed: %s" % e)

        try:
            state_msg = ModelState()    
            state_msg.model_name = 'unit_sphere_0_0'
            state_msg.pose.position.x = target[0]
            state_msg.pose.position.y = target[1]
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = -0.2
            state_msg.pose.orientation.w = 0
            self._svc_set_state(state_msg)
        except rospy.ServiceException as e:
            print("reset target failed: %s" % e)


    def run(self, hz=100):
        rospy.init_node(self._name)
        self.init_gazebo()
        self.init_vel()
        
        rate = rospy.Rate(hz)

        rospy.Subscriber('/'+self._name, Float64MultiArray, self.callback)
        rospy.Subscriber('/bridge_command', String, self.command_callback)
        
        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    ActionBridge().run()

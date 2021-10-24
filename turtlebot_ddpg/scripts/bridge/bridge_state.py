#! /usr/bin/env python


import threading
import numpy as np

import rospy
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist


class InfoGetter(object):
    def __init__(self, blocking=False, init_msg=None):
        if blocking:
            self._event = threading.Event()
        else:
            self._event = None
        self._msg = init_msg

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        self._msg = msg
        if self._event is not None:
            self._event.set()

    def get_msg(self, timeout=None):
        if self._event is not None:
            self._event.wait(timeout)
        return self._msg


class StateBridge(object):
    
    def __init__(self):
        self._name = 'bridge_state'
        self.position = Point(0,0,0)
        self.rotation = 0.0
        self.vel_ig = InfoGetter(init_msg=Twist())
        self.laser_ig = InfoGetter(blocking=True)


    def init_odom(self):
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("init_odom error: cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("init_odom error")

        self.get_odom()


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            self.position = Point(*trans)
            self.rotation = rotation[2]
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("get_odom error")
        
        return (self.position, self.rotation)


    def init_vel(self):
        self.vel_info = rospy.Subscriber("/cmd_vel", Twist, self.vel_ig)


    def get_vel(self):
        vel_msg = self.vel_ig.get_msg()
        return [vel_msg.linear.x, vel_msg.angular.z]


    def init_laser(self):
        self.laser_info = rospy.Subscriber("/laserscan_filtered", LaserScan, self.laser_ig)


    def get_laser(self):
        laser_msg = self.laser_ig.get_msg()
        normalized_laser = []
        for v in laser_msg.ranges:
            if v == float('Inf') or v > 3.5:
                normalized_laser.append(3.5)
            elif np.isnan(v):
                normalized_laser.append(0)
            else:
                normalized_laser.append(v)
        return normalized_laser

    
    def run(self, hz=100):
        rospy.init_node(self._name)
        self.init_odom()
        self.init_vel()
        self.init_laser()

        rate = rospy.Rate(hz)

        pub = rospy.Publisher('/'+self._name, Float64MultiArray, queue_size=1000)
        msg = Float64MultiArray()
        
        while not rospy.is_shutdown():
            pos, rot = self.get_odom()

            state = [pos.x, pos.y, pos.z, rot]
            state += self.get_vel()
            state += self.get_laser()
            msg.data = state
            pub.publish(msg)

            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    StateBridge().run()

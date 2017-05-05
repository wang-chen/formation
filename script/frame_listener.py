#!/usr/bin/env python
from numpy import *
import rospy
import tf
import math
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('require_tf')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    
    p = PointStamped()
    p.header.frame_id='uwb'
    p.point = Point(1,1,1)
    
     

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('ned', 'uwb', rospy.Time(0))
            p.header.stamp = rospy.Time(0)
            up = listener.transformPoint('ned', p)
            #print up
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
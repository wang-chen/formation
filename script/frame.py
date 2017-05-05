#!/usr/bin/env python
from matplotlib.lines import lineStyles
from numpy import *
from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter,AdditiveUnscentedKalmanFilter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import tf
import tf2_ros as tf2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualuwb.msg import uwb
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from numpy import radians as deg2rad
from numpy import degrees as rad2deg
import visualuwbfilter as vf
#from imusim.all import *
#from tf2_geometry.msg import transform_to_kdl
def viconcallback(msg):
    br = tf.TransformBroadcaster()
    t = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
    q = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    br.sendTransform(t, q , msg.header.stamp, "world","uav")
    
def uwbcallback(msg):
    #br = tf.TransformBroadcaster()
    ls = tf.TransformListener()
    p = PointStamped()
    p.header.frame_id = 'uwb'
    p.header.stamp = rospy.Time(0)
    p.point = msg.anchor
    
     
    while not rospy.is_shutdown():
        try:
            p_ned = ls.transformPoint('ned', p)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
    filter = vf.UWBLocation()
    Q  = zeros((x.shape[1],x.shape[1]))
    Q[0:3, 0:3] =  0.01*eye(3)
    Q[3:6, 3:6] =  0.000001*eye(3)
    Q[6:8, 6:8] =  0.0064*eye(2)
    Q[8,8]      =  0.0064
    Q[9,9]      =  0.0000001
    
    #xe[i+1], p[i+1] = filter.locate(, Q, 1.0*t/N, measure[i], anchor[i%4])

    #t = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
    #q = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
    #msg.header.stamp = rospy.Time.now()
    #msg.header.frame_id = 'vicon'
    #br.sendTransform(t, q , msg.header.stamp, "vicon","uav")
    

if __name__ == '__main__':
    rospy.init_node('uav_tf')
    
    #br  = tf2.TransformBroadcaster()
    br = tf.TransformBroadcaster()
    ls = tf.TransformListener()
        
    ned_tran = (0,0,0)
    ned_quar = quaternion_from_euler(0 ,pi,deg2rad(143-90))

    uwb_tran = (0,0,0)
    uwb_quar = quaternion_from_euler(0,0,deg2rad(143))

    rate = rospy.Rate(30.0)
    
    rospy.Subscriber("/vicon_node/mocap/pose", PoseStamped, viconcallback)
    rospy.Subscriber("/uwb_dis", uwb , uwbcallback)

    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        br.sendTransform(ned_tran,ned_quar,now,"ned","world")
        br.sendTransform(uwb_tran,uwb_quar,now,"uwb","ned")  
 
        rate.sleep()
        

        
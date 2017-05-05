#!/usr/bin/env python
__author__ = 'Jeffsan'
from VU_filter import *
from sslib import *

Q[ 0:3,  0:3] =  0.98*eye(3)#*10
Q[ 3:7,  3:7] =  0.01*eye(4)#*10
Q[ 7:9,  7:9] =  0.81*eye(2)#/2
Q[  9 ,   9 ] =  900#/2
Q[ 10 ,  10 ] =  0.000000001
#1.28 0.81 400 num = 2
#0.98 0.81,900 num =1
#0.5 0.01 100 num = 10
Q = Q*7
uwb = UWBLocation(0.01) 
uwb.setQ(Q)   
vision = VisionlLocation(1.0/100, K) 
vision.setQ(Q)
global xe,q,a,r,icount,uwbcount

def statecallback(msg):
    global xe,q,a,r,icount,uwbcount 
    pos        = array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    
   
    if icount % 2 == 0:
        anchor_pos = uwbanchor[uwbcount%4]
        y          = linalg.norm(pos - anchor_pos)+ random.randn(1,1)[0]*0.1
        xe, _ = uwb.locate(xe, Q, 1.0/(100), y, anchor_pos,q, a, r)
        uwbcount = uwbcount + 1
    else:
        visionmeasure = array([ dot(K, pos - visionanchor[i]) for i in xrange(4)]).reshape((8))+ random.randn(8) * 10
        xe, _ = vision.locate(xe, Q, 1.0/100, visionmeasure, visionanchor, q, a, r)
    
    icount     = icount+1
    br  = tf.TransformBroadcaster()
    uwb_tran = xe[0:3]
    uwb_q    = xe[3:7]
    br.sendTransform(uwb_tran, uwb_q, msg.header.stamp, "VisionUWB", "world")  


def imucallback(msg):
    global xe
    global q,a,r
    q  =  array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    r  =  array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.x])
    a  =  array([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z])


if __name__ == '__main__':

    rospy.init_node('uav_filter')
    rate = rospy.Rate(30.0)
    global xe,q,a,r,icount,uwbcount
    icount = 0
    uwbcount = 0
    xe = zeros((1,11))[0]
    xe[6] = 1
    xe[2] = 0.27
    q = array([0,0,0,1])
    a = array([0,0,0])
    r = array([0,0,0])
    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, statecallback, queue_size = 1, buff_size= 1,tcp_nodelay=True )
    rospy.Subscriber("/raw_imu", Imu, imucallback, queue_size = 1, buff_size= 1,tcp_nodelay=True)
    rospy.spin()

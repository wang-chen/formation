#!/usr/bin/env python
from sslib import *
from sensor_msgs.msg import Joy

edge = 1.0

uav0_t = (0, edge, 0)
uav0_q = quaternion_from_euler(0, 0, 0)

uav1_t = ( edge * sqrt(3)/2, -edge/2, 0)
uav1_q = quaternion_from_euler(0, 0, 0)

uav2_t = (-edge * sqrt(3)/2, -edge/2, 0)
uav2_q = quaternion_from_euler(0, 0, 0)

def callback(msg):
    ''' A: 0  B:1 X:2 Y:3
    '''
    global uav0_t, uav0_q, uav1_t, uav1_q, uav2_t, uav2_q
    
    if msg.buttons[0] == 1:#A button
        uav0_t = (0, edge, 0)
        uav0_q = quaternion_from_euler(0, 0, 0)
        
        uav1_t = ( edge * sqrt(3)/2, -edge/2, 0)
        uav1_q = quaternion_from_euler(0, 0, 0)
        
        uav2_t = (-edge * sqrt(3)/2, -edge/2, 0)
        uav2_q = quaternion_from_euler(0, 0, 0)
        rospy.loginfo("Change formation to Triangle!!")
        
    if msg.buttons[1] == 1:#A button
        uav0_t = (0, 0, 0)
        uav0_q = quaternion_from_euler(0, 0, 0)
        
        uav1_t = ( edge, 0, 0)
        uav1_q = quaternion_from_euler(0, 0, 0)
        
        uav2_t = (-edge, 0, 0)
        uav2_q = quaternion_from_euler(0, 0, 0)
        rospy.loginfo("Change formation to Straight Line!!")
        
    

if __name__ == '__main__':
    
    rospy.init_node('controller', anonymous=True)
        
    rospy.Subscriber("joy", Joy, callback)
    
    br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        br.sendTransform(uav0_t, uav0_q, now, '/target/uav0','/target/base_stabilized')
        br.sendTransform(uav1_t, uav1_q, now, '/target/uav1','/target/base_stabilized')
        br.sendTransform(uav2_t, uav2_q, now, '/target/uav2','/target/base_stabilized')
        rate.sleep()
        
        
        
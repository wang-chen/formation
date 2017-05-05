#!/usr/bin/env python
 
import rospy
import thread
import threading
import time
 
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from mavros.srv import CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
 
class Setpoint:
 
    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy
 
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
 
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"
 
        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/mavros/local_position/local', PoseStamped, self.reached)
 
    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz
 
        msg = PoseStamped()
        msg.header = Header() 
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time.now()
 
        while 1:
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
 
            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = Quaternion(*quaternion)
 
            self.pub.publish(msg)
 
            rate.sleep()
 
    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
 
        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()
 
        time.sleep(delay)
 
 
    def reached(self, topic):
            #print topic.pose.position.z, self.z, abs(topic.pose.position.z - self.z)
            if abs(topic.pose.position.x - self.x) < 0.2 and abs(topic.pose.position.y - self.y) < 0.2 and abs(topic.pose.position.z - self.z) < 0.2:
                self.done = True
            print "Current Pose:",topic.pose.position.x,topic.pose.position.y,topic.pose.position.z
            print "Set Pose:",self.x,self.y,self.z
            self.done_evt.set()
 
def setpoint_demo():
    pub = rospy.Publisher('/mavros/setpoint_position/local_position', PoseStamped, queue_size=10)
 
    rospy.init_node('pose', anonymous=True)
    rate = rospy.Rate(10) 
 
    setpoint = Setpoint(pub, rospy)
 
    print "move in x axis 1 meter "
    setpoint.set(0.0, 5.0, 1.2, 0)
    setpoint.set(5.0, 5.0, 1.2, 0)
    setpoint.set(5.0, 0.0, 1.2, 0)
    setpoint.set(0.0, 0.0, 1.2, 0)
 
 
    while not rospy.is_shutdown():
      print "NOT MANUAL" 
 
 
if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass

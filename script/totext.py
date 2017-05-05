#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3,Quaternion
filename = 'ground_truth.txt'
global q,a,r
a = Vector3(0,0,0)
r = Vector3(0,0,0)
q = Quaternion(0,0,0,1)

def statecallback(data):
    t = data.header.stamp
    p = data.pose.pose.position
    qt = data.pose.pose.orientation
    v = data.twist.twist.linear
    rt = data.twist.twist.angular
    global q,a,r

    f = open(filename,'a')
    f.write('%d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' % 
                                                (t.secs, t.nsecs, 
                                                 p.x, p.y, p.z,
                                                 qt.x, qt.y, qt.z, qt.w, 
                                                 v.x, v.y, v.z, 
                                                 rt.x, rt.y, rt.z,
                                                 q.x, q.y, q.z, q.w,
                                                 a.x, a.y, a.z,
                                                 r.x, r.y, r.z,
                                                 ))
    f.close()

def imucallback(data):
    global q,a,r
    q = data.orientation
    a = data.linear_acceleration
    r = data.angular_velocity

def listener():

    rospy.init_node('totext', anonymous=True)

    rospy.Subscriber("/ground_truth/state", Odometry, statecallback)
    rospy.Subscriber("/raw_imu", Imu, imucallback)

    rospy.spin()

if __name__ == '__main__':
    listener()

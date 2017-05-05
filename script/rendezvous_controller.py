#!/usr/bin/env python
from sslib import *
from formation.srv import Rendezvous

if __name__ == '__main__':
    
    rospy.init_node('controller', anonymous=True)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0.2
    msg.angular.z = 0
    pub.publish(msg)
    
    vis_pub = rospy.Publisher(sys.argv[1]+'arrow', Marker,  queue_size=0);
    marker = Marker()
    marker.header.frame_id = '/' + sys.argv[1] + '/base_stabilized'
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.scale.x = 1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    
    rate = rospy.Rate(10)
    rospy.sleep(2)
    for i in xrange(20):
        pub.publish(msg)
        rate.sleep()
    
    msg.linear.z = 0
    pub.publish(msg)
    
    print sys.argv[1] + " is waiting for rendezvous service..."
    rospy.wait_for_service('rendezvous_service')
    hunt = rospy.ServiceProxy('rendezvous_service', Rendezvous)
    print sys.argv[1] + " got service!"
    
    
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            pose0,pose1,pose2,pose3 = Pose(),Pose(),Pose(),Pose()
            (T0, Q0) = listener.lookupTransform('/world', '/target/'+sys.argv[1], rospy.Time(0))       
            (T1, Q1) = listener.lookupTransform('/world', '/'+sys.argv[1]+'/base_stabilized', rospy.Time(0))
            (T2, Q2) = listener.lookupTransform('/world', '/'+sys.argv[2]+'/base_stabilized', rospy.Time(0))
            (T3, Q3) = listener.lookupTransform('/world', '/'+sys.argv[3]+'/base_stabilized', rospy.Time(0))
            
            pose0.position.x,  pose0.position.y,  pose0.position.z = T0[0], T0[1], T0[2]
            pose0.orientation.x, pose0.orientation.y, pose0.orientation.z, pose0.orientation.w = Q0[0], Q0[1], Q0[2], Q0[3]
            
            pose1.position.x,  pose1.position.y,  pose1.position.z = T1[0], T1[1], T1[2]
            pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w = Q1[0], Q1[1], Q1[2], Q1[3]
            
            pose2.position.x,  pose2.position.y,  pose2.position.z = T2[0], T2[1], T2[2]
            pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w = Q2[0], Q2[1], Q2[2], Q2[3]
            
            pose3.position.x,  pose3.position.y,  pose3.position.z = T3[0], T3[1], T3[2]
            pose3.orientation.x, pose3.orientation.y, pose3.orientation.z, pose3.orientation.w = Q3[0], Q3[1], Q3[2], Q3[3]
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print sys.argv[1]+" cant't get translation rotation!"
            continue

        poses = [pose0, pose1, pose2, pose3]
        res = hunt(poses)

        twist = res.twist[1]
        twist.linear.x =  twist.linear.x * 0.3
        twist.linear.y =  twist.linear.y * 0.3
        twist.linear.z =  (pose0.position.z + 0.4 - pose1.position.z )*0.3
        
        marker.header.stamp = rospy.Time.now();
        quat = quaternion_from_euler(0, 0, atan2(twist.linear.y, twist.linear.x))
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        vis_pub.publish(marker);
        pub.publish(twist)
        rate.sleep()
        

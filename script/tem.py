#!/usr/bin/env python
from sslib import *
from visualuwb.srv import Rendezvous
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.wait_for_service('rendezvous_service')
    try:
        print "requiring.."
        hunt = rospy.ServiceProxy('rendezvous_service', Rendezvous)
        #req = Rendezvous()
          
        poses = []
        
        pose = Pose() 
        pose.position.x=0
        pose.position.y=0
        pose.orientation.w=1
        poses.append(pose)
        
        pose = Pose() 
        pose.position.x=-1
        pose.position.y=0
        pose.orientation.w=1
        poses.append(pose)
        
        pose = Pose() 
        pose.position.x=-1
        pose.position.y=1
        pose.orientation.w=1
        poses.append(pose)
        
        pose = Pose() 
        pose.position.x=-1
        pose.position.y=-1
        pose.orientation.w=1
        poses.append(pose)
        
        res = hunt(poses)
        
        print res

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

   
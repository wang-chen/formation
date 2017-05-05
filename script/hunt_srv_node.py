#!/usr/bin/env python
from sslib import *
from formation.srv import *
from HuntController import *
        
def server(req):
        
    hunt = HuntController()
    
    return hunt.decide(req.pose)


if __name__ == '__main__':
    
    rospy.init_node('rendezvous_server', anonymous = True)
   
    s = rospy.Service('rendezvous_service', Rendezvous, server)
    
    print "Ready to serve for rendezvous server"
    
    rospy.spin()
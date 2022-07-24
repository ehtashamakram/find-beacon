#!/usr/bin/env python

from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
import math


def node():
    pub = rospy.Publisher('shapes', Marker, queue_size=10)
    rospy.init_node('plotter', anonymous=False)
    rate = rospy.Rate(1)
   

    points=Marker()
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id="/odom"
    points.header.stamp=rospy.Time.now()

    points.id = 0

    points.type = Marker.POINTS
#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action =  Marker.ADD;

    points.pose.orientation.w =  1.0;

    points.scale.x=0.03; 
    points.scale.y=0.03; 

    points.color.g = 1.0;
   
    points.color.a= 1.0;
    points.lifetime =rospy.Duration();

    p=Point()
    p.x = 1;
    p.y = 1;
    p.z = 1;

    pp=[]
    
    pp.append(copy(p))
       
    while not rospy.is_shutdown():
        p.x+=0.1  
        pp.append(copy(p)) 
        points.points=pp
        #print points.points,'\n','----------------','\n'

        pub.publish(points)
        rate.sleep()
#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import numpy as np
from sensor_msgs.msg import*

def nothing(msg):
	pass


def image_callback(msg):
	bridge = cv_bridge.CvBridge()
	image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

	cv2.namedWindow('Colorbars')
	cv2.namedWindow('orignal')
	frame = cv2.resize(image, (720, 480)) # Resize image
	#assign strings for ease of coding
	hh='Hue High'
	hl='Hue Low'
	sh='Saturation High'
	sl='Saturation Low'
	vh='Value High'
	vl='Value Low'
	wnd = 'Colorbars'
	#Begin Creating trackbars for each
	cv2.createTrackbar(hl, wnd,0,179,nothing)
	cv2.createTrackbar(hh, wnd,0,179,nothing)
	cv2.createTrackbar(sl, wnd,0,255,nothing)
	cv2.createTrackbar(sh, wnd,0,255,nothing)
	cv2.createTrackbar(vl, wnd,0,255,nothing)
	cv2.createTrackbar(vh, wnd,0,255,nothing)
	#it is common to apply a blur to the frame
	frame=cv2.GaussianBlur(frame,(5,5),0)
	 
	#convert from a BGR stream to an HSV stream
	hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#read trackbar positions for each trackbar
	hul=cv2.getTrackbarPos(hl, wnd)
	huh=cv2.getTrackbarPos(hh, wnd)
	sal=cv2.getTrackbarPos(sl, wnd)
	sah=cv2.getTrackbarPos(sh, wnd)
	val=cv2.getTrackbarPos(vl, wnd)
	vah=cv2.getTrackbarPos(vh, wnd)
	 
	#make array for final values
	HSVLOW=np.array([hul,sal,val])
	HSVHIGH=np.array([huh,sah,vah])
	 
	#create a mask for that range
	mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
	res = cv2.bitwise_and(frame,frame, mask =mask)


	 
	cv2.imshow(wnd, res)
	cv2.imshow('orignal', image)
	cv2.waitKey(3)

		
	 
	#cv2.destroyAllWindows()

rospy.init_node('follower')
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
rospy.spin()
#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import numpy as np
import rospkg
from sensor_msgs.msg import*

def image_callback(msg):
		
		bridge = cv_bridge.CvBridge()
		rospack = rospkg.RosPack()
		target_colour = rospy.get_param("/target_colour")
		save_path = rospack.get_path('team30')
		cv2.namedWindow("window", 1)
		image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		image = cv2.resize(image, (720, 480)) # Resize image
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 29, 86, 6])
		upper_yellow = numpy.array([64, 255, 255])
		# define the list of boundaries
		boundaries = [([0, 200, 0], [0, 255, 255], "red"),
						([ 36, 196, 95], [114, 255, 255], "green"),
						([100,150,0], [140,255,255], "blue"),
						([29, 225, 100], [37, 255, 255], "yellow") ]
		# loop over the boundaries
		for (lower, upper, code) in boundaries:
			# create NumPy arrays from the boundaries
			lower = np.array(lower)
			upper = np.array(upper)
			# find the colors within the specified boundaries and apply
			# the mask
			mask = cv2.inRange(hsv, lower, upper)
			h, w, d = image.shape
			M = cv2.moments(mask)
			if M['m00'] > 0:
									res = cv2.bitwise_and(image,image, mask =mask)
									res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
									ret,thresh = cv2.threshold(res,127,255,0)
									contours = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
									if len(contours) != 0:
										cnt = contours[0]
										area = cv2.contourArea(cnt)
										if code == target_colour and area > 8000:
											print "saving"
											cv2.imwrite(save_path+"/snaps/the_beacon.jpg", image)
										if code == "yellow" and area > 10000:
											print str(code)+" beacon is detected"
										if code == "red" and area > 10000:
											print str(code)+" beacon is detected"
											print area
										if code == "blue" and area > 10000:
											print str(code)+" beacon is detected"
										if code == "green" and area > 10000:
											print str(code)+" beacon is detected"
										else:
											print "Searching for Beacon"



			
		
		cv2.imshow("window", image)
		cv2.waitKey(3)

rospy.init_node('beacon_detection')

image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
rospy.spin()
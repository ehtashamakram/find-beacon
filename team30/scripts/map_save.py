#!/usr/bin/env python

import rospkg
import roslaunch
import rospy

rospack = rospkg.RosPack()
save_path = rospack.get_path('team30')
path = str(save_path)+"/launch/map.launch"

rospy.init_node('en_Mapping', anonymous=True)

while not rospy.is_shutdown():
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid, [save_path+"/launch/map.launch"])
  launch.start()
  rospy.loginfo("started")
  rospy.sleep(5)

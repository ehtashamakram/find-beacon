#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time
import roslaunch

x_axis = 0
y_axis = 0
task= 0
indication=None
is_Frontier_Point=False
queuem=[]
map_open_list=[]
map_close_list=[]
frontier_open_list=[]
frontier_close_list=[]

def callback(data):
        global x_axis, y_axis, task, move_base
        valid = False
        front_map = OccupancyGrid()

        while valid is False:
            map_size = randrange(len(data.data))
            front_map = data.data[map_size]
            queuem = front_map
            edges = wave_forentier_check(data, map_size)
            if front_map != -1 and front_map <= 0.2 and edges is True:
                valid = True
            
        row = map_size / 384
        col = map_size % 384

        x_axis = col * 0.05 - 10  # column * resolution + origin_x
        y_axis = row * 0.05 - 10  # row * resolution + origin_x
        
        if task% 2 == 0:
            task += 1
            # Start the robot moving toward the goal
            goto_goal()

def wave_forentier_check(data, map_size):
  global indication, map_open_list, is_Frontier_Point, queuem
  global map_open_list, map_close_list, frontier_open_list
  global frontier_close_list

  map_neighbour=[[-1,0],[1,0],[0,-1],[0,1]]
  while(len(queuem)>0):
    p=queuem.pop(0)
    if p.indication==map_close_list:
      continue
    if p.is_Frontier_Point==True:
      queuef=[]
      New_Frontier=[]
      queuef.append(p)
      p.indication==frontier_open_list
      while(len(queuef>0)):
        q=queuef.pop(0)
        if q.indication==map_close_list or q.indication==frontier_open_list:
          continue
        if q.is_Frontier_Point==True:
          New_Frontier.append(q)
          for i in range(3):
            offset=neighbour[i]
            new_position=[q.x+offset[0],q.y+offset[1]]
            if new_position.indication!=frontier_open_list and new_position.indication!=frontier_open_list and new_position.indication!=map_close_list:
              queuef.append(new_position)
              new_position.indication==frontier_open_list
        q.indication==frontier_open_list
        map_close_list=New_Frontier
        for point in New_Frontier:
          queuem = 0
          point.indication==map_close_list


  unknowns = 0
  obstacles = 0
  for i in range(-3, 4):
            for j in range(-3, 4):
                row = i * 384 + j
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
            if unknowns > 0 and obstacles < 2:
                return True
            else:
                return False
    

def goto_goal():
        global move_base
        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_axis
        goal.target_pose.pose.position.y = y_axis
        goal.target_pose.pose.orientation.w = 1.0
        move_base.send_goal(goal, check_goal_status)


def check_goal_status(status, result):
        """ Check the status of a goal 
        """
        global task
        task += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("our Goal is succe")


        # Goal aborted
        if status == 4:
            rospy.loginfo("our Goal is aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("our Goal is rejected")


if __name__ == '__main__':
    try:
        rospy.init_node('explore', log_level=rospy.DEBUG)
        move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 
        sub_map = rospy.Subscriber('/map', OccupancyGrid, callback)
        rospy.spin()
    except:
        rospy.ROSInterruptException
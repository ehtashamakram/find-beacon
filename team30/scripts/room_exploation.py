#!/usr/bin/env python
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time
from geometry_msgs.msg import Twist


class Explore:

    def __init__(self):
        """ Initialize environment
        """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))

        self.x = 0
        self.y = 0
        self.completion = 0

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.count = 0
        self.PI = 3.1415926535897
        self.vel_msg = Twist()
        time.sleep(8)


    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        valid = False

        while valid is False:
            map_size = randrange(len(data.data))
            self.map = data.data[map_size]

            edges = self.check_neighbors(data, map_size)
            if self.map != -1 and self.map <= 0.2 and edges is True:
                valid = True
            
        row = map_size / 384
        col = map_size % 384

        self.x = col * 0.1 - 20  # column * resolution + origin_x
        self.y = row * 0.1 - 20  # row * resolution + origin_x
        
        if self.completion % 2 == 0:
            self.completion += 1
            # Start the robot moving toward the goal
            self.set_goal()
    

    def set_goal(self):
        """ Set goal position for move_base.
        """

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(goal, self.goal_status)


    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion += 1

        # Goal reached
        if status == 3:
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            while(current_angle < 20):
                t1 = rospy.Time.now().to_sec()
                current_angle = t1-t0
                self.vel_msg.angular.z = 0.5
                self.velocity_publisher.publish(self.vel_msg)
            self.vel_msg.angular.z = 0
            self.velocity_publisher.publish(self.vel_msg)


    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
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


def main():
    """ The main() function """
    Explore()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('explore', log_level=rospy.DEBUG)
    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server()
    try:
        main()
    except:
        rospy.ROSInterruptException
#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point

from bark_msgs.msg import CameraProjectionsAction, CameraProjectionsGoal

if __name__ == '__main__':
    rospy.init_node('camera_projections_client')

    client = actionlib.SimpleActionClient('camera_projections', CameraProjectionsAction)
    client.wait_for_server()

    goal = CameraProjectionsGoal()
    p = Point()
    p.x = int(1080/2)
    p.y = int(1920/2)
    p.z = 0.2
    goal.points.append(p)
    goal.space_type = goal.IMAGE_2D
    goal.header.frame_id = 'base'
    
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())
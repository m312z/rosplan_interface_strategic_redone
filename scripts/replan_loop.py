#!/usr/bin/env python
import rospkg
import rospy
import sys
import time
import os

from std_srvs.srv import Empty, EmptyResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse

rospy.init_node("coordinator")

print "waiting for things to start"
rospy.sleep(5.)

# rospy.wait_for_service( '/strategic/rosplan_interface_strategic_control/decompose_problem')
# pd = rospy.ServiceProxy('/strategic/rosplan_interface_strategic_control/decompose_problem', Empty)
# if not pd():
# 	print('CANNOT DECOMPOSE')

goal_achieved = False
replans = 0
while not goal_achieved and replans<25:
    rospy.wait_for_service('/strategic/strategic_problem_interface/problem_generation_server')
    rospy.wait_for_service('/strategic/strategic_planner_interface/planning_server')
    rospy.wait_for_service('/strategic/strategic_parsing_interface/parse_plan')
    rospy.wait_for_service('/strategic/strategic_plan_dispatch/dispatch_plan')
    
    try:
        pg = rospy.ServiceProxy('/strategic/strategic_problem_interface/problem_generation_server', Empty)
        if not pg():
            print('NO PROBLEM')

        pi = rospy.ServiceProxy('/strategic/strategic_planner_interface/planning_server', Empty)
        if not pi():
            rospy.sleep(5.)
            replans += 1
        else:
            pp = rospy.ServiceProxy('/strategic/strategic_parsing_interface/parse_plan', Empty)
            pp()
            rospy.sleep(1.)

            dp = rospy.ServiceProxy('/strategic/strategic_plan_dispatch/dispatch_plan', DispatchService)
            dsr = dp()

            goal_achieved = dsr.goal_achieved
            if not dsr.goal_achieved:
                print('failed. replan~~~~~~~')
                replans += 1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        replans += 1

if replans>=25:
    print "Please email a human, I am stuck."
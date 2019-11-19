#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from math import sqrt

def robot_at(msg, params, robot_name):
    assert(msg.header.frame_id == "map")
    assert(len(params) == 1)
    ret_value = []
    attributes = get_kb_attribute(robot_name+"_at")
    curr_wp = ''
    # Find current robot_location
    for a in attributes:
        if not a.is_negative:
            curr_wp = a.values[0].value
            break

    distance = float('inf')
    closest_wp = ''
    for wp in params[0]:
        pose = rospy.get_param("/ground_wp/"+wp)
        assert(len(pose) > 0)
        x = pose[0] - msg.pose.pose.position.x
        y = pose[0] - msg.pose.pose.position.y
        d = sqrt(x**2 + y**2)
        if d < distance:
            closest_wp = wp
            distance = d
    if curr_wp != closest_wp:
        ret_value.append((curr_wp, False)) # Set current waypoint to false
        ret_value.append((closest_wp, True))  # Set new wp to true
    return ret_value

def ugv0_at(msg, params):
    print "ugv0"
    return robot_at(msg, params, "ugv0")

def ugv1_at(msg, params):
    print "ugv1"
    return robot_at(msg, params, "ugv1")

def ugv2_at(msg, params):
    print "ugv2"
    return robot_at(msg, params, "ugv2")


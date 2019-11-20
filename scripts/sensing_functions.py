#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
            if curr_wp == "uav01":
                "current", a
            break

    distance = float('inf')
    closest_wp = ''
    for wp in params[0]:
        pose = rospy.get_param("/ground_wp/"+wp)
        assert(len(pose) > 0)
        x = pose[0] - msg.pose.pose.position.x
        y = pose[1] - msg.pose.pose.position.y
        d = sqrt(x**2 + y**2)
        if d < distance:
            closest_wp = wp
            distance = d
    if curr_wp != closest_wp:
        if closest_wp == "uav01":
            print "closest"
        if curr_wp != '':
            ret_value.append((curr_wp, False)) # Set current waypoint to false
        if closest_wp != '':
            ret_value.append((closest_wp, True))  # Set new wp to true
    return ret_value


def ugv0_at(msg, params):
    return robot_at(msg, params, "ugv0")


def ugv1_at(msg, params):
    return robot_at(msg, params, "ugv1")


def ugv2_at(msg, params):
    return robot_at(msg, params, "ugv2")


def uav_at(msg, params):
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    ret_value = []
    attributes = get_kb_attribute("uav_at")

    for uav in params[0]:
        curr_wp = ''
        # Find current robot_location
        for a in attributes:
            if not a.is_negative and a.values[0].value==uav:
                curr_wp = a.values[1].value

        distance = float('inf')
        closest_wp = ''
        for wp in params[1]:
            pose = rospy.get_param("/sky_wp/"+wp)
            assert(len(pose) > 0)
            x = pose[0] - (msg.pose.pose.position.x + 20.3777)
            y = pose[1] - (msg.pose.pose.position.y - 6.75525)
            z = pose[2] - (msg.pose.pose.position.z)
            d = sqrt(x**2 + y**2 + z**2)
            if d < distance and pose[2]>1:
                closest_wp = wp
                distance = d
        if curr_wp != closest_wp:
            if curr_wp != '':
                ret_value.append((uav+':'+curr_wp, False)) # Set current waypoint to false
            if closest_wp != '':
                ret_value.append((uav+':'+closest_wp, True))  # Set new wp to true
    return ret_value

occupancy_map = {}
def not_occupied(msg, params, publisher):
    distance = 2 #float('inf')
    closest_wp = ''
    ret_value = []
    for wp in params[0]:
        pose = rospy.get_param("/ground_wp/"+wp)
        assert(len(pose) > 0)
        x = pose[0] - (msg.pose.pose.position.x)
        y = pose[1] - (msg.pose.pose.position.y)
        z = pose[2] - (msg.pose.pose.position.z)
        d = sqrt(x**2 + y**2 + z**2)
        if not wp in occupancy_map:
            occupancy_map[wp] = []
        if d < distance and not publisher in occupancy_map[wp]:
            occupancy_map[wp].append(publisher)
        elif d > distance and publisher in occupancy_map[wp]:
            occupancy_map[wp].remove(publisher)
        ret_value.append((wp, len(occupancy_map[wp])==0))
    return ret_value

    




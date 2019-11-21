# return format for predicates/functions
# ret = []
#
# The resturn value is a list of changes.
# each change has the format "(key, value)"
#
# Key is a string in the format:
# "param[0]:param[1]:...:param[n]"
# For propositions: value is Bool.
# For functions: value is Real.
#
# Example for proposition (distance group_wp0 ground_wp45) to be set to 56.78:
# ret.append( ( "group_wp0:ground_wp45", 56.78 ) )
#
# return ret

#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
import numpy as np
import heapq

def robot_at(msg, params, robot_name):
    assert(msg.header.frame_id == "map")
    assert(len(params) == 1)
    ret_value = []
    attributes = get_kb_attribute(robot_name+"_at")
    curr_wp = ''

    # check for and add new sky wps
    ground_wps = rospy.get_param("/ground_wp")
    if len(params[0]) != len(ground_wps):
        del params[0][:]
        for wp in ground_wps:
            if type(ground_wps[wp]) is list:
                params[0].append(wp)

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
        y = pose[1] - msg.pose.pose.position.y
        d = sqrt(x**2 + y**2)
        if d < distance:
            closest_wp = wp
            distance = d
    if curr_wp != closest_wp:
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

    # check for and add new sky wps
    sky_wps = rospy.get_param("/sky_wp")
    if len(params[1]) != len(sky_wps):
        del params[1][:]
        for wp in sky_wps:
            if type(sky_wps[wp]) is list:
                params[1].append(wp)

    # update UAV location
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
            if d < distance and msg.pose.pose.position.z>2:
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



def close_point(a, b, max_distance):
    if not type(a) is list or not type(b) is list:
        return False
    count = 0
    for (p,q) in zip(a,b):
        if (not type(p) is float and not type(p) is int) or (not type(q) is float and not type(q) is int):
            return False
        if abs(p-q)>max_distance:
            return False
        count = count + 1
        if count == 2:
            break
    return True


def dijkstra(wp_start, wp_goal, prm_nodes, prm_edges): 

    open_list = prm_nodes.keys()
    
    dist_list = dict(zip(prm_nodes.keys(), [float('inf')]*len(prm_nodes)))
    dist_list[wp_start] = 0

    priority_queue = zip([float('inf')]*len(prm_nodes), prm_nodes.keys())
    priority_queue.remove((float('inf'),wp_start))
    priority_queue.append((float(0),wp_start))
    heapq.heapify(priority_queue)
    
    # Set the initial node as current.
    next_pair = heapq.heappop(priority_queue)
    current_node = next_pair[1]
    open_list.remove(current_node)

    while len(open_list)>0:

        # update the distances of the neighbours
        for child in prm_edges[current_node]:

            if not child in open_list:
                continue

            d = dist_list[current_node] + prm_edges[current_node][child]
            if d < dist_list[child]:
                heapq.heappush(priority_queue, (d, child))
                dist_list[child] = d

        # get next current node
        next_pair = heapq.heappop(priority_queue)
        while not next_pair[1] in open_list:
            next_pair = heapq.heappop(priority_queue)
        if next_pair[0]==float('inf'):
            break
        elif next_pair[1]==wp_goal:
            return next_pair[0]
        current_node = next_pair[1]
        open_list.remove(current_node)

    return -1

def check_visibility(ground_wp, sky_wp):
    return close_point(ground_wp, sky_wp, 4)

def distance(msg, params):

    distance = float('inf')
    ret_value = []

    # create KB update client
    rospy.wait_for_service('/strategic/rosplan_knowledge_base/update_array')
    rospy.wait_for_service('/tactical_execution/rosplan_knowledge_base/update_array')
    rospy.wait_for_service('/strategic/rosplan_interface_strategic_control/add_mission_goal')
    update_kb = rospy.ServiceProxy('/strategic/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    update_kb_tactical = rospy.ServiceProxy('/tactical_execution/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    update_strategic_goals = rospy.ServiceProxy('/strategic/rosplan_interface_strategic_control/add_mission_goal', KnowledgeUpdateServiceArray)

    # add new ground_wp to the KB
    ground_wps = rospy.get_param("/ground_wp")
    try:
        pos = [float(msg.data[0]),float(msg.data[1]),0.0,0.0]
        for wp in ground_wps:

            # check if wp is the new one
            if close_point(pos,ground_wps[wp],0.01):

                # upload new wp to KB
                kus = KnowledgeUpdateServiceArrayRequest()
                kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                ki = KnowledgeItem()
                ki.instance_type = "ground"
                ki.instance_name = wp
                ki.knowledge_type = ki.INSTANCE
                kus.knowledge.append(ki)
                res = update_kb(kus)
                res = update_kb_tactical(kus)

                # Oberve added waypoint (if not original)
                if len(wp)>=10 and int(wp[9:])>4:
                    kus = KnowledgeUpdateServiceArrayRequest()
                    kus.update_type += np.array(kus.ADD_GOAL).tostring()
                    ki = KnowledgeItem()
                    ki.knowledge_type = ki.FACT
                    ki.attribute_name = 'observed'
                    kv = KeyValue()
                    kv.key = 'wp'
                    kv.value = wp
                    ki.values.append(kv)
                    kus.knowledge.append(ki)
                    try:
                        res = update_strategic_goals(kus)
                    except:
                        print "Something Happened"

                # add new wp to the prm
                prm_nodes = rospy.get_param("/prm/wp")
                prm_edges = rospy.get_param("/prm/edge")

                rospy.set_param("/prm/wp/"+wp, ground_wps[wp])
                prm_nodes[wp] = ground_wps[wp]
                prm_edges[wp] = {}
                for node in prm_nodes:
                    if node==wp:
                        continue
                    d = sqrt( (ground_wps[wp][0]-prm_nodes[node][0])*(ground_wps[wp][0]-prm_nodes[node][0])
                            + (ground_wps[wp][1]-prm_nodes[node][1])*(ground_wps[wp][1]-prm_nodes[node][1])
                            + (ground_wps[wp][2]-prm_nodes[node][2])*(ground_wps[wp][2]-prm_nodes[node][2]) )
                    if d <= 4:
                        rospy.set_param("/prm/edge/"+wp+"/"+node, d)
                        rospy.set_param("/prm/edge/"+node+"/"+wp, d)
                        prm_edges[wp][node] = d
                        prm_edges[node][wp] = d

                # add new distances for each other wp                
                kus = KnowledgeUpdateServiceArrayRequest()
                for other_wp in ground_wps:
                    if other_wp == wp:
                        continue

                    # don't check frame_id
                    if not type(ground_wps[other_wp]) is list:
                        continue

                    # run dijkstra
                    distance = dijkstra(wp, other_wp, prm_nodes, prm_edges)

                    if distance > 0:
                        # upload distances to KB
                        ki = KnowledgeItem()
                        ki.knowledge_type = ki.FUNCTION
                        ki.attribute_name = "distance"
                        kv = KeyValue()
                        kv.key = 'wp1'
                        kv.value = other_wp
                        ki.values.append(kv)
                        kv = KeyValue()
                        kv.key = 'wp2'
                        kv.value = wp
                        ki.values.append(kv)
                        ki.function_value = distance
                        kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                        kus.knowledge.append(ki)

                        ki = KnowledgeItem()
                        ki.knowledge_type = ki.FUNCTION
                        ki.attribute_name = "distance"
                        kv = KeyValue()
                        kv.key = 'wp1'
                        kv.value = wp                    
                        ki.values.append(kv)
                        kv = KeyValue()
                        kv.key = 'wp2'
                        kv.value = other_wp
                        ki.values.append(kv)
                        ki.function_value = distance
                        kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                        kus.knowledge.append(ki)
                res = update_kb(kus)
                res = update_kb_tactical(kus)

                # add new visibility between sky and ground
                sky_wps = rospy.get_param("/sky_wp")
                added_visibility = False

                kus = KnowledgeUpdateServiceArrayRequest()
                for sky_wp in sky_wps:
                    if check_visibility(ground_wps[wp], sky_wps[sky_wp]):

                        ki = KnowledgeItem()
                        ki.knowledge_type = ki.FACT
                        ki.attribute_name = "can_observe"
                        kv = KeyValue()
                        kv.key = 'wp1'
                        kv.value = sky_wp                    
                        ki.values.append(kv)
                        kv = KeyValue()
                        kv.key = 'wp2'
                        kv.value = wp
                        ki.values.append(kv)
                        kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                        kus.knowledge.append(ki)

                        added_visibility = True

                if added_visibility:
                    res = update_kb(kus)
                    res = update_kb_tactical(kus)

                # create new sky_wp if no visibility was created
                if not added_visibility:

                    new_sky_wp_name = "sky_above_"+wp

                    # upload new wp to KB
                    kus = KnowledgeUpdateServiceArrayRequest()
                    kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                    ki = KnowledgeItem()
                    ki.instance_type = "sky"
                    ki.instance_name = new_sky_wp_name
                    ki.knowledge_type = ki.INSTANCE
                    kus.knowledge.append(ki)
                    res = update_kb(kus)
                    res = update_kb_tactical(kus)

                    # create new sky_wp and add to param server
                    new_sky_wp = list(ground_wps[wp])
                    new_sky_wp[2] = new_sky_wp[2] + 8
                    rospy.set_param("/sky_wp"+new_sky_wp_name, new_sky_wp)

                    # add visibility of other ground wps for the new sky wp
                    kus = KnowledgeUpdateServiceArrayRequest()
                    for gwp in ground_wps:

                        if check_visibility(ground_wps[gwp], new_sky_wp):

                            ki = KnowledgeItem()
                            ki.knowledge_type = ki.FACT
                            ki.attribute_name = "can_observe"
                            kv = KeyValue()
                            kv.key = 'wp1'
                            kv.value = new_sky_wp_name                
                            ki.values.append(kv)
                            kv = KeyValue()
                            kv.key = 'wp2'
                            kv.value = wp
                            ki.values.append(kv)
                            kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                            kus.knowledge.append(ki)

                    if len(kus.knowledge) > 0:
                        res = update_kb(kus)
                        res = update_kb_tactical(kus)

                break
    except rospy.ServiceException, e:
        rospy.logerr("KCL (%s) Failed to update knowledge base: %s" % rospy.get_name(), e.message)

    return ret_value


    




#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_interface_mapping.srv import CreatePRM
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Float32MultiArray

# get path of pkg
rospy.init_node("map_setup")

# load parameters
max_prm_size = rospy.get_param('~max_prm_size', 1000)

# wait for services
rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
wp_pub = rospy.Publisher('/uav01/add_waypoint', Float32MultiArray, queue_size=100)


# generate dense PRM
rospy.loginfo("KCL: (%s) Creating PRM of size %i" % (rospy.get_name(), max_prm_size))
prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
if not prm(max_prm_size,2,3,4,50,100000):
    rospy.logerr("KCL: (%s) No PRM was made" % rospy.get_name())

# load ground_wps from parameter
ground_wps = rospy.get_param("/ground_wp")

# add each as a new waypoint to the PMR (usin sensing interface)
for wp in ground_wps:
	if type(ground_wps[wp]) is list:
		rospy.loginfo("KCL: (%s) Adding wp: %s" % (rospy.get_name(), wp))
		wpmsg = Float32MultiArray()
		for coord in ground_wps[wp]:
			wpmsg.data.append(coord)
		wp_pub.publish(wpmsg)
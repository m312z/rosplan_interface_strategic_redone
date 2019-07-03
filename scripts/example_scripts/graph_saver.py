#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback_strategic(data):
    f1 = open("strategic_plan.dot", "w")
    f1.write(data.data)
    f1.close()

def callback_tactical(data):
    f2 = open("tactical_plan.dot", "w")
    f2.write(data.data)
    f2.close()

def listener():
    rospy.init_node('graph_saver', anonymous=True)
    rospy.Subscriber("/strategic_plan_dispatch/plan_graph", String, callback_strategic)
    rospy.Subscriber("/tactical_plan_dispatch/plan_graph", String, callback_tactical)
    rospy.spin()

if __name__ == '__main__':
    listener()

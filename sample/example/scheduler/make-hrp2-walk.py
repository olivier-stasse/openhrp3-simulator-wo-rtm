#!/usr/bin/python
import sys
import rospy

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *


# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    rospy.loginfo("Stack of Tasks launched")

    runCommandClient("from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize")
    runCommandClient("solver=initialize(robot)")

    runCommandStartDynamicGraph()

    runCommandClient("from dynamic_graph.sot.pattern_generator.walking import CreateEverythingForPG, walkFewSteps")
    runCommandClient("CreateEverythingForPG(robot,solver)")
    runCommandClient("walkFewSteps(robot)")

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)


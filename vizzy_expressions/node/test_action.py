#! /usr/bin/env python

import roslib
import rospy

import actionlib

import vizzy_expressions.msg

rospy.init_node('test_action_expression')

client = actionlib.SimpleActionClient('ExpressionDriver', vizzy_expressions.msg.ExpressionAction)

client.wait_for_server()

goal = vizzy_expressions.msg.ExpressionGoal();

goal.mode = goal.PREDEFINED;
goal.emotion = goal.FACE_EVIL;
goal.subsystem = goal.PART_ALL;

print 'sending goal'

client.send_goal(goal);
client.wait_for_result();

goal.mode = goal.PREDEFINED;
goal.emotion = goal.FACE_SURPRISED;
goal.subsystem = goal.PART_ALL;

print 'sending goal'

client.send_goal(goal);
client.wait_for_result();


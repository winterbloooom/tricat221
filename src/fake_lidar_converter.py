#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tricat221.msg import FakeSensor
from sensor_msgs.msg import LaserScan
from tricat221.msg import Obstacle, ObstacleList

global ob_cases
global pub

ob_cases = []
ob_cases.append()
ob_cases.append()
ob_cases.append()
ob_cases.append()
ob_cases.append()

def callback(msg):
    global ob_cases
    global pub

    if msg.sensor_type != 3:
        pass
    pub.publish(ob_cases[msg.ob_case - 1])

# pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
pub = rospy.Publisher("/obstacles", ObstacleList, queue_size=10)
rospy.Subscriber("/sensing_value", FakeSensor, callback, queue_size=1)
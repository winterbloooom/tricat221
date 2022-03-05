#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
from tricat221.msg import FakeSensor

global pub
global h

def heading_publish():
    global pub
    global h

    while True:
        pub.publish(h)
        rospy.sleep(0.2)

def callback(msg):
    global h

    if msg.sensor_type == 2:
        h = msg.heading_ang
        # heading_publish(msg.heading_ang)

rospy.init_node('F_headingCalculator', anonymous=False)

pub = rospy.Publisher("/heading", Float64, queue_size=0)
rospy.Subscriber("/sensing_value", FakeSensor, callback, queue_size=1)

while not rospy.is_shutdown():
    global h
    pub.publish(h)
    rospy.sleep(0.2)

rospy.spin()
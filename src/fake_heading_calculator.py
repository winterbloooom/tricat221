#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
from tricat221.msg import FakeSensor

class FakeHeading:
    def __init__(self):
        self.h = 0
        self.pub = rospy.Publisher("/heading", Float64, queue_size=0)
        rospy.Subscriber("/sensing_value", FakeSensor, self.callback, queue_size=1)

    def callback(self, msg):
        if msg.sensor_type == 2:
            self.h = msg.heading_ang

rospy.init_node('F_headingCalculator', anonymous=False)

fake_heading = FakeHeading()
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    fake_heading.pub.publish(fake_heading.h)

rospy.spin()
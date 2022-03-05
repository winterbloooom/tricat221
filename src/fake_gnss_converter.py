#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tricat221.msg import FakeSensor
from geometry_msgs.msg import Point

class FakeGNSS:
    def __init__(self):
        self.x, self.y = 0, 0
        self.pub = rospy.Publisher('enu_position', Point, queue_size=10)
        rospy.Subscriber("/sensing_value", FakeSensor, self.callback, queue_size=1)
    
    def callback(self, msg):
        if msg.sensor_type == 1:
            self.x, self.y = msg.enu_pos[0], msg.enu_pos[1]

rospy.init_node('F_gnssConverter', anonymous=False)
fake_gnss = FakeGNSS()
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    fake_gnss.pub.publish(Point(fake_gnss.x, fake_gnss.y, 0))

rospy.spin()
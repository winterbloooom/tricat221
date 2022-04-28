#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64

class Par:
    def __init__(self):
        self.sub = rospy.Subscriber("/toyou", Float64, self.callback, queue_size=5)
        self.data = None

    def callback(self, msg):
        self.data = msg.data

    def print_par(self):
        print("P ", self.data)


class Child(Par):
    def __init__(self):
        super(Child, self).__init__()
    
    def callback(self, msg):
        self.data = msg.data

    def print_chi(self):
        rospy.loginfo("C ", self.data)

rospy.init_node("listen")

par = Par()
chi = Child()

while not rospy.is_shutdown():
    par.print_par()
    chi.print_chi()

    rospy.spin()
#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tricat221.msg import FakeSensor
from geometry_msgs.msg import Point

global pub

def enu_publish(x, y):
    global pub

    while True:
        pub.publish(Point(x, y, 0))
        rospy.sleep(0.2)

def callback(msg):
    # global x
    # global y

    if msg.sensor_type == 1:
        enu_publish(msg.enu_pos[0], msg.enu_pos[1])
    # x = msg.enu_pos[0]
    # y = msg.enu_pos[1]


rospy.init_node('F_gnssConverter', anonymous=False)
pub = rospy.Publisher('enu_position', Point, queue_size=10)
rospy.Subscriber("/sensing_value", FakeSensor, callback, queue_size=1)

rospy.spin()

# while not rospy.is_shutdown():
#     global x
#     global y

#     pub.publish(Point(x, y, 0))

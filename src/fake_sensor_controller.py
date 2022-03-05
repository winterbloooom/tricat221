#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tricat221.msg import FakeSensor

rospy.init_node("F_Controller")

pub = rospy.Publisher('sensing_value', FakeSensor, queue_size=0)

while not rospy.is_shutdown():
    fake_sensor = FakeSensor()
    sensor_type = int(input("Sensor (1: GPS / 2: IMU / 3: LiDAR) >> "))
    fake_sensor.sensor_type = sensor_type
    if sensor_type == 1:
        x = input("ENU Pos X >> ")
        y = input("ENU Pos Y >> ")
        fake_sensor.enu_pos = [x, y]#list(int(str(input("ENU Pos (input: x/y) >> ")).split("/")))

    elif sensor_type == 2:
        fake_sensor.heading_ang = int(input("Heading Ang (deg) >> "))
    else:
        fake_sensor.ob_case = int(input("Scan Case (1~5) >> "))
    pub.publish(fake_sensor) # 한 번에 못 받으면?
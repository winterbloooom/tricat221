#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

origin = None
boat = None

def gps_fix_callback(msg):
    global boat
    boat = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, origin[0], origin[1], origin[2])

def enu_convert(gnss, origin):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return [e, n]

def main():
    global origin

    rospy.init_node("gnss_converter", anonymous=True)

    while True:
        msg = rospy.wait_for_message("/ublox_gps/fix", NavSatFix)
        # 시간 지정 안하면 block 함수임
        origin = [msg.latitude, msg.longitude, msg.altitude]
        if not origin == None:
            break

    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    enu_position = Point()

    while not rospy.is_shutdown():
        if boat == None:
            continue
        enu_position.x = boat[0]
        enu_position.y = boat[1]

        rospy.Publisher("origin", NavSatFix, queue_size=10).publish(msg)
        pub.publish(enu_position)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

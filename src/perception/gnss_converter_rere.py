#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

origin = rospy.get_param("origin")
boat = None

def gps_fix_callback(msg):
    global boat
    boat = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, origin[0], origin[1], origin[2])

def enu_convert(gnss, dxdy):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return [e-dxdy[0], n-dxdy[1]]

def main():
    rospy.init_node("gnss_converter", anonymous=True)

    msg = rospy.wait_for_message("/ublox_gps/fix", NavSatFix)
    now_origin_xy = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, origin[0], origin[1], origin[2])[:2]
    print(now_origin_xy)
    dx = -now_origin_xy[0]
    dy = -now_origin_xy[1]

    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    enu_position = Point()
    dxdy = Point()

    while not rospy.is_shutdown():
        if boat == None:
            continue
        enu_position.x = boat[0]
        enu_position.y = boat[1]

        dxdy.x = dx
        dxdy.y = dy

        rospy.Publisher("dxdy", Point, queue_size=10).publish(dxdy)
        pub.publish(enu_position)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

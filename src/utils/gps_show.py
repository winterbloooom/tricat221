#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual
from visualization_msgs.msg import MarkerArray



origin = rospy.get_param("origin")
boat = [0, 0]
msg_in = [0, 0]
l1 = [35.069359, 128.5789284, 49.0]
l2 = [35.0696497, 128.5788137, 49.0]
l3 = [35.069664, 128.5788595, 49.0]
l4 = [35.0696782, 128.5789079, 49.0]
l5 = [35.0693834, 128.5790279, 49.0]
d1 = [35.0696287, 128.578821, 49.0]
d2 = [35.0696596, 128.5789156, 49.0]

def gps_fix_callback(msg):
    msg_in[0], msg_in[1] = msg.latitude, msg.longitude
    boat[0], boat[1] = enu_convert([msg.latitude, msg.longitude, msg.altitude])


def enu_convert(gnss):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return e, n

def main():
    l1_re_y, l1_re_x  = enu_convert(l1)
    l2_re_y, l2_re_x  = enu_convert(l2)
    l3_re_y, l3_re_x  = enu_convert(l3)
    l4_re_y, l4_re_x  = enu_convert(l4)
    l5_re_y, l5_re_x  = enu_convert(l5)
    d1_re_y, d1_re_x  = enu_convert(d1)
    d2_re_y, d2_re_x  = enu_convert(d2)

    rospy.init_node("gnss_converter", anonymous=True)

    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)
    visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

    rate = rospy.Rate(10)  # 10Hz

    enu_position = Point()

    while not rospy.is_shutdown():
        enu_position.x = boat[0]
        enu_position.y = boat[1]
        pub.publish(enu_position)
        b_x, b_y = boat[1], boat[0]

        # visualize
        ids = list(range(0, 100))

        l1_p = visual.point_rviz(name="fixed", id=ids.pop(), x=l1_re_x, y=l1_re_y, color_g=255, scale=0.2)
        l2_p = visual.point_rviz(name="fixed", id=ids.pop(), x=l2_re_x, y=l2_re_y, color_r=255, scale=0.2)
        l3_p = visual.point_rviz(name="fixed", id=ids.pop(), x=l3_re_x, y=l3_re_y, color_r=255, scale=0.2)
        l4_p = visual.point_rviz(name="fixed", id=ids.pop(), x=l4_re_x, y=l4_re_y, color_r=255, scale=0.2)
        l5_p = visual.point_rviz(name="fixed", id=ids.pop(), x=l5_re_x, y=l5_re_y, color_r=255, scale=0.2)
        d1_p = visual.point_rviz(name="fixed", id=ids.pop(), x=d1_re_x, y=d1_re_y, color_b=255, scale=0.2)
        d2_p = visual.point_rviz(name="fixed", id=ids.pop(), x=d2_re_x, y=d2_re_y, color_b=255, scale=0.2)

        new_point = visual.point_rviz(name="point", id=0, x=b_x, y=b_y, color_g=255)
        enu_txt = visual.text_rviz(
            name="point",
            id=ids.pop(),
            x=b_x, y=b_y,
            text="({:>4.2f}, {:>4.2f})\n({}, {})".format(b_x, b_y, msg_in[0], msg_in[1]),
        )
        all_markers = visual.marker_array_rviz(
            [
                l1_p, l2_p, l3_p, l4_p, l5_p, d1_p, d2_p, new_point, enu_txt
            ]
        )
        visual_rviz_pub.publish(all_markers)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
import cv2

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from visualization_msgs.msg import MarkerArray

import utils.visualizer as visual

origin = rospy.get_param("origin")
boat = [0, 0, 0]
msg_in = [0, 0, 0]
l1 = [35.069359, 128.5789284, 49.0]
l2 = [35.0696497, 128.5788137, 49.0]
l3 = [35.069664, 128.5788595, 49.0]
l4 = [35.0696782, 128.5789079, 49.0]
l5 = [35.0693834, 128.5790279, 49.0]
d1 = [35.0696287, 128.578821, 49.0]
d2 = [35.0696596, 128.5789156, 49.0]

s1 = [35.0693012, 128.5789177, 49.0]
s2 = [35.0696497, 128.5788137, 49.0]
s3 = [35.069664, 128.5788595, 49.0]
s4 = [35.0696782, 128.5789079, 49.0]
s5 = [35.0693834, 128.5790279, 49.0]

cv2.namedWindow("controller")
cv2.createTrackbar("x", "controller", 80, 160, lambda x: x)
cv2.createTrackbar("y", "controller", 80, 160, lambda x: x)


def gps_fix_callback(msg):
    msg_in[0], msg_in[1], msg_in[2] = msg.latitude, msg.longitude, msg.altitude
    boat[0], boat[1], boat[2] = enu_convert([msg.latitude, msg.longitude, msg.altitude])


def enu_convert(gnss):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    # print(u)
    return [n, e, u]

def geodetic_convert(enu):
    e = enu[1]
    n = enu[0]
    u = enu[2]
    lat, lon, alt = pm.enu2geodetic(e, n, u, origin[0], origin[1], origin[2])
    return [lat, lon, alt]

def get_trackbar_pos():
    x = (cv2.getTrackbarPos("x", "controller") - 80) / 2.0
    y = (cv2.getTrackbarPos("y", "controller") - 80) / 2.0
    return x, y

def main():
    l1_re = enu_convert(l1)
    l2_re = enu_convert(l2)
    l3_re = enu_convert(l3)
    l4_re = enu_convert(l4)
    l5_re = enu_convert(l5)
    d1_re = enu_convert(d1)
    d2_re = enu_convert(d2)

    l1_gn = geodetic_convert(l1_re)

    rospy.init_node("gnss_converter", anonymous=True)

    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)
    visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

    rate = rospy.Rate(10)  # 10Hz

    enu_position = Point()

    while not rospy.is_shutdown():
        x, y = get_trackbar_pos()
        picked_gn = geodetic_convert([x, y, -0.1])

        enu_position.x = boat[0]
        enu_position.y = boat[1]
        pub.publish(enu_position)

        # visualize
        ids = list(range(0, 100))

        picked_point = visual.point_rviz(
            name="point", id=ids.pop(), x=x, y=y, color_b=255, color_g=255, scale=0.3
        )
        picked_point_txt = visual.text_rviz(
            name="point",
            id=ids.pop(),
            x=x, y=y,
            text="({}, {})\n({}, {})".format(x, y, picked_gn[0], picked_gn[1]),
        )

        l1_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=l1_re[0], y=l1_re[1], color_g=255, scale=0.25
        )
        l1_txt = visual.text_rviz(
            name="fixed",
            id=ids.pop(),
            x=l1_re[0], y=l1_re[1],
            text="Origin\n({}, {})\n({}, {})".format(l1[0], l1[1], l1_re[0], l1_re[1]),
        )
        l2_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=l2_re[0], y=l2_re[1], color_r=255, scale=0.25
        )
        l3_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=l3_re[0], y=l3_re[1], color_r=255, scale=0.25
        )
        l4_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=l4_re[0], y=l4_re[1], color_r=255, scale=0.25
        )
        l5_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=l5_re[0], y=l5_re[1], color_r=255, scale=0.25
        )
        d1_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=d1_re[0], y=d1_re[1], color_r=255, scale=0.25
        )
        d2_p = visual.point_rviz(
            name="fixed", id=ids.pop(), x=d2_re[0], y=d2_re[1], color_r=255, scale=0.25
        )
        boundary = visual.linelist_rviz(
            name="boundary",
            id=ids.pop(),
            lines=[l1_re, l2_re, l2_re, l4_re, l4_re, l5_re, l5_re, l1_re],
            color_r=65,
            color_g=53,
            color_b=240,
            scale=0.15,
        )

        # station = visual.linelist_rviz(
        #     name="boundary",
        #     id=ids.pop(),
        #     lines=[d1_re, l2_re, d2_re, l4_re],
        #     color_r=65,
        #     color_g=53,
        #     color_b=240,
        #     scale=0.25,
        # )

        axis_x = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[-40, 0], [40, 0]],
            color_r=255,
            scale=0.1,
        )
        axis_y = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[0, -40], [0, 40]],
            color_g=255,
            scale=0.1,
        )


        new_point = visual.point_rviz(name="point", id=0, x=boat[0], y=boat[1], color_g=255)

        enu_txt = visual.text_rviz(
            name="point",
            id=ids.pop(),
            x=boat[0],
            y=boat[1],
            text="({:>4.2f}, {:>4.2f})\n({}, {})".format(boat[0], boat[1], msg_in[0], msg_in[1]),
        )

        all_markers = visual.marker_array_rviz(
            [picked_point, picked_point_txt, l1_p, l1_txt, boundary, 
            l2_p, l3_p, l4_p, l5_p, d1_p, d2_p, 
            new_point, enu_txt, axis_x, axis_y]
        )
        visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""Visualize GPS data with Rviz

Attributes:
    origin (list): 
        Origin coordinate(latitude, longitude, altitude) data with DD.MMMMMMM format. 
        Get this data from /tricat221/params/coordinates.ymal (e.g. [35.0695517, 128.5788733, 49.0] )
    enu_coord (list):
        Coordinate data list to subscribe and save current location with ENU coordinate system. 
        Raw GNSS data is converted using "origin".
    geodetic_coord (list):
        Coordinate data list to subscribe and save current location with DD.MMMMMMM format.

Note:
    * Launch file: /tricat221/launch/sensor_test.launch의 <!-- gps_show --> 부분 주석 해제 후 사용
    * What can you do with this module
        * Draw the boundary
        * Draw lacation and watch coordinates of each points in ENU or geodetic(dd.mmmm)
        * Pick specific point
        * Draw current input data
"""

import os
import sys
import cv2
import pymap3d as pm
import rospy
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual
import utils.gnss_converter as gc


# set module attributes
origin = rospy.get_param("origin")
enu_coord = [0, 0, 0] # N(x), E(y), U(z)
geodetic_coord = [0, 0, 0]

# set trackbar
cv2.namedWindow("controller")
cv2.createTrackbar("x", "controller", 100, 200, lambda x: x)
cv2.createTrackbar("y", "controller", 100, 200, lambda x: x)

def gps_fix_callback(msg):
    """Subscribe geodetic coordinate from GPS and save both geodetic and ENU coordinates

    Args:
        msg (NavSatFix): geodetic coordinate including latitude, longitude and altitude value
    """
    geodetic_coord[0], geodetic_coord[1], geodetic_coord[2] = msg.latitude, msg.longitude, msg.altitude
    enu_coord[0], enu_coord[1], enu_coord[2] = gc.enu_convert(geodetic_coord)

def geodetic_convert(enu):
    """Convert ENU coordinate to geodetic(dd.mmmmmm) system
    
    Args:
        enu (list): 
            ENU coordinate to convert to DD.MMMMMM format. 
            U(Up, latitude) value is not accurate, just arbitrarily picked (ususally 0 or -1)

    Returns:
        lat, lon, alt (float): Converted latitude, longitude, altitude coordinate from "origin"
    """
    lat, lon, alt = pm.enu2geodetic(enu[0], enu[1], enu[2], origin[0], origin[1], origin[2])
    return lat, lon, alt


def get_trackbar_pos():
    """Get trackbar position to set the point location(x, y)(ENU)

    Returns:
        x, y (float): the location of picked point
    
    Notes:
        * scale = 0.5 meters
        * the trackbar value "0" means "-50 meters", "100" means "0 meters", and "200" means "+50 meters"
    """
    x = (cv2.getTrackbarPos("x", "controller") - 100) / 2.0
    y = (cv2.getTrackbarPos("y", "controller") - 100) / 2.0
    return x, y


def main():
    # set boundary vertices
    boundary1_geo = rospy.get_param("boundary1")
    boundary2_geo = rospy.get_param("boundary2")
    boundary1_enu = []
    boundary2_enu = []
    for p in boundary1_geo:
        boundary1_enu.append(list(gc.enu_convert(p)))
    for p in boundary2_geo:
        boundary2_enu.append(list(gc.enu_convert(p)))

    # ros nodes
    rospy.init_node("gnss_show", anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

    while not rospy.is_shutdown():
        # get & set picked point info
        x, y = get_trackbar_pos()
        picked_geo = geodetic_convert([x, y, 0])

        # rviz
        ids = list(range(0, 100))

        boundary1 = visual.linelist_rviz(
            name="boundary",
            id=ids.pop(),
            lines=[
                boundary1_enu[0], boundary1_enu[1], 
                boundary1_enu[1], boundary1_enu[2],
                boundary1_enu[2], boundary1_enu[3],
                boundary1_enu[3], boundary1_enu[0]
                ],
            color_r=59,
            color_g=196,
            color_b=212,
            scale=0.15,
        )

        boundary2 = visual.linelist_rviz(
            name="boundary",
            id=ids.pop(),
            lines=[
                boundary2_enu[0], boundary2_enu[1], 
                boundary2_enu[1], boundary2_enu[2],
                boundary2_enu[2], boundary2_enu[3],
                boundary2_enu[3], boundary2_enu[0]
                ],
            color_r=59,
            color_g=196,
            color_b=212,
            scale=0.15,
        )

        axis_x = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[-50, 0], [50, 0]],
            color_r=255,
            scale=0.1,
        )
        axis_y = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[0, -50], [0, 50]],
            color_g=255,
            scale=0.1,
        )

        current_point = visual.point_rviz(name="point", id=0, x=enu_coord[0], y=enu_coord[1], color_g=255, scale=0.5)
        current_txt = visual.text_rviz(
            name="point",
            id=ids.pop(),
            x=enu_coord[0],
            y=enu_coord[1],
            text="({:>4.2f}, {:>4.2f})\n({}, {})".format(enu_coord[0], enu_coord[1], geodetic_coord[0], geodetic_coord[1]),
            scale=0.8
        )

        picked_point = visual.point_rviz(
            name="point", id=ids.pop(), x=x, y=y, color_r=255, color_g=255, color_b=255, scale=0.5
        )
        picked_point_txt = visual.text_rviz(
            name="point",
            id=ids.pop(),
            x=x,
            y=y,
            text="({:>4.2f}, {:>4.2f})\n({}, {})".format(x, y, picked_geo[0], picked_geo[1]),
            scale=0.8
        )

        all_markers = visual.marker_array_rviz(
            [
                boundary1,
                boundary2,
                current_point,
                current_txt,
                picked_point,
                picked_point_txt,
                axis_x,
                axis_y,
            ]
        )

        for idx in range(len(boundary1_enu)):
            point = visual.point_rviz(
                name="waypoints",
                id=ids.pop(),
                x=boundary1_enu[idx][0],
                y=boundary1_enu[idx][1],
                color_r=78,
                color_g=166,
                color_b=58,
                scale=0.3,
            )
            visual.marker_array_append_rviz(all_markers, point)

            point_txt = visual.text_rviz(
                name="waypoints",
                id=ids.pop(),
                x=boundary1_enu[idx][0],
                y=boundary1_enu[idx][1],
                text="({:>6.4f}, {:>6.4f})\n({:>8.6f}, {:>9.6f})".format(boundary1_enu[idx][0], boundary1_enu[idx][1], boundary1_geo[idx][0], boundary1_geo[idx][1]),
            )
            visual.marker_array_append_rviz(all_markers, point_txt)

        for idx in range(len(boundary2_enu)):
            point = visual.point_rviz(
                name="waypoints",
                id=ids.pop(),
                x=boundary2_enu[idx][0],
                y=boundary2_enu[idx][1],
                color_r=78,
                color_g=166,
                color_b=58,
                scale=0.3,
            )
            visual.marker_array_append_rviz(all_markers, point)

            point_txt = visual.text_rviz(
                name="waypoints",
                id=ids.pop(),
                x=boundary2_enu[idx][0],
                y=boundary2_enu[idx][1],
                text="({:>6.4f}, {:>6.4f})\n({:>8.6f}, {:>9.6f})".format(boundary2_enu[idx][0], boundary2_enu[idx][1], boundary2_geo[idx][0], boundary2_geo[idx][1]),
            )
            visual.marker_array_append_rviz(all_markers, point_txt)

        visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()

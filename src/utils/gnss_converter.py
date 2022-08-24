#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""Subscribe GNSS data and publish its converted data in ENU coordinate system

GNSS Converter Node & function
===========================================================================

.. ENU Coordinate system:
    https://en.wikipedia.org/wiki/Axes_conventions#Ground_reference_frames:_ENU_and_NED

.. pymap3d library:
    https://pypi.org/project/pymap3d/

Attributes:
    origin (list): 
        Origin coordinate(latitude, longitude, altitude) data with DD.MMMMMMM format. 
        Get this data from /tricat221/params/coordinates.ymal (e.g. [35.0695517, 128.5788733, 49.0] )
    boat (list):
        Coordinate data list to subscribe and save current location with ENU coordinate system. 
        Raw GNSS data is converted using "origin".

Notes:
    Using "pymap3d" library, we can convert GNSS datas with various formats. 
    In this module, geodetic coordinates(lat, lon, al) -> ENU coordinates(e, n, u)
"""

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

origin = rospy.get_param("origin")
boat = [0, 0, 0]


def gps_fix_callback(msg):
    """Subscribe geodetic coordinate from GPS and convert in ENU coordinate

    Args:
        msg (NavSatFix): geodetic coordinate including latitude, lonagitude and altitude value
    """
    boat[0], boat[1], boat[2] = enu_convert([msg.latitude, msg.longitude, msg.altitude])


def enu_convert(gnss):
    """Convert in ENU coordinate using pymap3d

    Args:
        gnss (list): a list including [latitude, longitude, altitude]

    Returns:
        e (float): ENU East coordinate (meters) from "origin"
        n (float): ENU North coordinate (meters) from "origin"

    Note:
        In i-Tricat221 coordinate system, the x-axis is magnetic North.
        So, return [n, e, u] not [e, n, u]
    """
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return n, e, u


def main():
    rospy.init_node("gnss_converter", anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    enu_position = Point()

    while not rospy.is_shutdown():
        enu_position.x = boat[0]
        enu_position.y = boat[1]

        pub.publish(enu_position)

        rate.sleep()


if __name__ == "__main__":
    main()

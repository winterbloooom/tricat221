#!/usr/bin/env python
# -*- coding:utf-8 -*-

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def basic_setting(name, id, color_r, color_g, color_b, color_a=1):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, color_a)
    marker.pose.orientation.w = 1.0
    return marker


def point_rviz(name, id, x, y, color_r=0, color_g=0, color_b=0, scale=0.1):
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.POINTS
    marker.scale = Vector3(scale, scale, 0)
    marker.points.append(Point(x, y, 0))
    return marker


def points_rviz(name, id, points, color_r=0, color_g=0, color_b=0, scale=0.1):
    # points = [[x, y], [x, y], ...]
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.POINTS
    marker.scale = Vector3(scale, scale, 0)
    for point in points:
        marker.points.append(Point(point[0], point[1], 0))
    return marker


def arrow_rviz(name, id, x1, y1, x2, y2, color_r=0, color_g=0, color_b=0, scale_x=0.2, scale_y=0.4):
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.ARROW
    marker.scale = Vector3(scale_x, scale_y, 0)
    marker.points.append(Point(x1, y1, 0))
    marker.points.append(Point(x2, y2, 0))
    return marker


def text_rviz(name, id, x, y, text, scale=0.5):
    marker = basic_setting(name, id, color_r=255, color_g=255, color_b=255)
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose.position = Point(x, y, 0)
    marker.scale.z = scale
    marker.text = text
    return marker


def linelist_rviz(name, id, lines, color_r=0, color_g=0, color_b=0, scale=0.05):
    # lines = [[begin_x, begin_y], [end_x, end_y], [...], ...]
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.LINE_LIST
    marker.scale.x = scale
    for line in lines:
        marker.points.append(Point(line[0], line[1], 0))
    return marker

def cylinder_rviz(name, id, x, y, scale, color_r=0, color_g=0, color_b=0):
    marker = basic_setting(name, id, color_r, color_g, color_b, color_a=0.6)
    marker.type = Marker.CYLINDER
    marker.scale = Vector3(scale, scale, 0.01)
    marker.pose.position = Point(x, y, 0)
    return marker

def marker_array_rviz(markers):
    marker_array = MarkerArray()
    for marker in markers:
        marker_array.markers.append(marker)
    return marker_array

def marker_array_append_rviz(marker_array, marker):
    marker_array.markers.append(marker)
    return marker_array
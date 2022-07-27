#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""visualize components using Rviz

Notes:
    * frame_id = /map
"""

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def basic_setting(name, id, color_r, color_g, color_b, color_a=1):
    """make Marker object with basic settings

    Args:
        * name (str) : Marker name. It can share same name with others
        * id (int) : Marker id. It should be unique.
        * color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)
        * color_a (int) : Transparency (0 ~ 255)

    Returns:
        Marker : Marker object with basic settings

    Notes:
        * set frame_id, namespace, id, action, color, orientation
        * ColorRGBA는 0~1사이 값을 사용하므로 편의상 0~255 단위로 입력받아 여기서 255로 나누어줌
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, color_a)
    marker.pose.orientation.w = 1.0
    return marker


def point_rviz(name, id, x, y, color_r=0, color_g=0, color_b=0, scale=0.1):
    """make one Point Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        x, y (float, float) : point position in meter
        color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)
        scale (float) : size of point in meter

    Returns:
        Marker : Point Marker object
    """
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.POINTS
    marker.scale = Vector3(scale, scale, 0)
    marker.points.append(Point(x, y, 0))
    return marker


def points_rviz(name, id, points, color_r=0, color_g=0, color_b=0, scale=0.1):
    """make a set of Point Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        points (list) : point positions in meter ([[x, y], [x, y], ...])
        color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)
        scale (float) : size of point in meter

    Returns:
        Marker : Point Marker object
    """
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.POINTS
    marker.scale = Vector3(scale, scale, 0)
    for point in points:
        marker.points.append(Point(point[0], point[1], 0))
    return marker


def arrow_rviz(name, id, x1, y1, x2, y2, color_r=0, color_g=0, color_b=0, scale_x=0.2, scale_y=0.4):
    """make a Arrow Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        x1, y1, x2, y2 (float) : start and end point of arrow in meter
        color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)
        scale_x, scale_y (float) : size of arrow in meter

    Returns:
        Marker : Arrow Marker object
    """
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.ARROW
    marker.scale = Vector3(scale_x, scale_y, 0)
    marker.points.append(Point(x1, y1, 0))  # tail
    marker.points.append(Point(x2, y2, 0))  # head
    return marker


def text_rviz(name, id, x, y, text, scale=0.5):
    """make a Text Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        x, y (float) : position of text in meter
        text (str) : text to write
        scale (float) : size of text in meter

    Returns:
        Marker : Point Marker object
    """
    marker = basic_setting(name, id, color_r=255, color_g=255, color_b=255)
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose.position = Point(x, y, 0)
    marker.scale.z = scale
    marker.text = text
    return marker


def linelist_rviz(name, id, lines, color_r=0, color_g=0, color_b=0, scale=0.05):
    """make a Line List Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        lines (list) : set of lines' each end positions. [[begin_x, begin_y], [end_x, end_y], [begin_x, begin_y], [end_x, end_y], ...]
        color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)
        scale (float) : thickness of Line List in meter

    Returns:
        Marker : Line List Marker object
    """
    marker = basic_setting(name, id, color_r, color_g, color_b)
    marker.type = Marker.LINE_LIST
    marker.scale.x = scale
    for line in lines:
        marker.points.append(Point(line[0], line[1], 0))
    return marker


def cylinder_rviz(name, id, x, y, scale, color_r=0, color_g=0, color_b=0):
    """make a Cylinder Marker

    Args:
        name (str) : Marker name. It can share same name with others
        id (int) : Marker id. It should be unique.
        x, y (float) : position of cylinder center in meter
        scale (float) : diameter of cylinder
        color_r, color_g, color_b (int) : Marker color in RGB format (0 ~ 255)

    Returns:
        Marker : Cylinder Marker object
    """
    marker = basic_setting(name, id, color_r, color_g, color_b, color_a=0.6)
    marker.type = Marker.CYLINDER
    marker.scale = Vector3(scale, scale, 0.01)
    marker.pose.position = Point(x, y, 0)
    return marker


def marker_array_rviz(markers):
    """make a MarkerArray object

    Args:
        markers (list) : list of Marker objects. [marker, marker, ...]

    Returns:
        MarkerArray : MarkerArray object having input markers
    """
    marker_array = MarkerArray()
    for marker in markers:
        marker_array.markers.append(marker)
    return marker_array


def marker_array_append_rviz(marker_array, marker):
    """append one Marker object in exisiting MarkerArray object

    Args:
        marker_array (MarkerArray) : MarkerArray object
        marker (Marker) : Marker objects to append

    Returns:
        MarkerArray : MarkerArray object
    """
    marker_array.markers.append(marker)
    return marker_array

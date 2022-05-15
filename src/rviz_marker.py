#!/usr/bin/env python
#-*- coding:utf-8 -*-

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
import sys

class Arrow:
    def __init__(self, ns, id, color, scale, p1, p2):
        self.mark = Marker()
        self.mark.header.frame_id = '/laser' #map'
        self.mark.ns = ns
        self.mark.id = id
        self.mark.type = Marker.ARROW
        self.mark.action = Marker.ADD
        self.mark.color = ColorRGBA(color[0], color[1], color[2], color[3])
        self.mark.scale = Vector3(scale[0], scale[1], scale[2])
        self.mark.pose.orientation.w = 1
        self.mark.points.append(Point(p1[0], p1[1], 0))
        self.mark.points.append(Point(p2[0], p2[1], 0))


class LineStrip:
    def __init__(self, ns="range", id=0, color=[1, 1, 1, 1], scale_x=0.08, p1=[0, 0], p2=[0, 0]):
        self.mark = Marker()
        self.mark.header.frame_id = '/laser' #map'
        self.mark.ns = ns
        self.mark.id = id
        self.mark.type = Marker.LINE_STRIP
        self.mark.action = Marker.ADD
        self.mark.color = ColorRGBA(color[0], color[1], color[2], color[3])
        self.mark.scale.x = scale_x
        self.mark.pose.orientation.w = 1        
        self.mark.points.append(Point(p1[0], p1[1], 0))
        self.mark.points.append(Point(p2[0], p2[1], 0))

class Text:
    def __init__(self, ns, id, text, pose):
        self.mark = Marker()
        self.mark.header.frame_id = '/laser' #map'
        self.mark.ns = ns
        self.mark.id = id
        self.mark.type = Marker.TEXT_VIEW_FACING
        self.mark.action = Marker.ADD
        self.mark.color = ColorRGBA(1, 1, 1, 1)
        self.mark.scale.z = 0.5
        self.mark.text = text
        self.mark.pose.position = Point(pose[0], pose[1], 0)

class Points:
    def __init__(self, ns, id, color, scale, p):
        self.mark = Marker()
        self.mark.header.frame_id = '/laser' #map'
        self.mark.ns = ns
        self.mark.id = id
        self.mark.type = Marker.POINTS
        self.mark.action = Marker.ADD
        self.mark.color = ColorRGBA(color[0], color[1], color[2], color[3])
        self.mark.scale = Vector3(scale[0], scale[1], 0)
        self.mark.points.append(Point(p[0], p[1], 0))
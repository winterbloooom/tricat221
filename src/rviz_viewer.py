#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RvizMarker:
    def __init__(self, name, marker_id, marker_type, point_scale=0.1, r=1, g=1, b=1):
        self.marker = Marker()

        # 고정값
        self.marker.header.frame_id = '/map'
        self.marker.action = 0 #ADD
        self.color_a = 1.0 # 투명도 03
        
        # 파라미터로 받는 값
        self.marker.ns = name
        self.marker.id = marker_id
        self.marker.type = marker_type
        self.color_r, self.color_g, self.color_b = r, g, b

        self.scale_x = point_scale
        self.scale_y = point_scale

    def append_marker_point(self, x, y):
        self.marker.points.append(Point(x, y, 0))

    def return_marker(self):
        return self.marker
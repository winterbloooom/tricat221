#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

class RvizMarker:
    def __init__(self, name, marker_id, marker_type, p_scale=0.1, a_x_scale=0.2, a_y_scale=0.4, r=0, g=0, b=0):
        self.marker = Marker()

        # 고정값
        self.marker.header.frame_id = '/map'
        self.marker.action = 0 #ADD
        self.marker.color.a = 1.0 # 투명도
        self.marker.pose.orientation.w = 1.0
        
        # 파라미터로 받는 값
        self.marker.ns = name
        self.marker.id = marker_id
        self.marker.type = marker_type
            # 8: POINT
        self.marker.color.r, self.marker.color.g, self.marker.color.b = r, g, b

        if marker_type == 0: #arrow
            self.marker.scale = Vector3(a_x_scale, a_y_scale, 0)
        else:
            self.marker.scale = Vector3(p_scale, p_scale, 0)

    def append_marker_point(self, x, y):
        self.marker.points.append(Point(x, y, 0))
        

    def return_marker(self):
        return self.marker
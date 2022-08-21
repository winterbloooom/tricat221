#!/usr/bin/env python
# -*- coding:utf-8 -*-

from math import atan2, cos, degrees, pow, sin, sqrt

# TODO 정리

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def polar_to_cartesian(cls, r, phi):
        return cls(r * cos(phi), r * sin(phi))

    @classmethod
    def empty_point(cls):
        return cls(0, 0)

    def __add__(self, p2):
        return Point(self.x + p2.x, self.y + p2.y)

    def __sub__(self, p2):
        return Point(self.x - p2.x, self.y - p2.y)

    def __mul__(self, c):
        return Point(self.x * c, self.y * c)

    def __div__(self, d):
        return Point(self.x / d, self.y / d) if d != 0 else Point(0, 0)

    def __eq__(self, p2):
        return self.x == p2.x and self.y == p2.y

    def dist_from_origin(self):
        return sqrt(pow(self.x, 2.0) + pow(self.y, 2.0))

    def dist_squared_from_origin(self):
        return pow(self.x, 2.0) + pow(self.y, 2.0)

    def dist_btw_points(self, p2):
        return sqrt(pow(self.x - p2.x, 2.0) + pow(self.y - p2.y, 2.0))

    def dist_btw_points2(self, p2_x, p2_y):
        return sqrt(pow(self.x - p2_x, 2.0) + pow(self.y - p2_y, 2.0))

    def polar_angle_rad(self):
        return atan2(self.y, self.x)

    def polar_angle_deg(self):
        return degrees(atan2(self.y, self.x))

    def dot(self, p):
        return self.x * p.x + self.y * p.y

    def cross(self, p):
        return self.x * p.y - self.y * p.x

    def normalized_point(self):
        return (self / self.dist_from_origin()) if self.dist_from_origin() > 0.0 else self

    def reflected(self, normal_vec):
        return self - normal_vec * (2 * normal_vec.dot(self))

    def perpendicular(self):
        return Point(-self.y, self.x)

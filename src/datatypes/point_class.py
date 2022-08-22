#!/usr/bin/env python
# -*- coding:utf-8 -*-

from math import atan2, cos, degrees, pow, sin, sqrt

"""lidar_converter 등에 사용할 '한 점' 혹은 '한 벡터' 정의 클래스"""


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def polar_to_cartesian(cls, r, phi):
        """극좌표계 표현(각도, 거리) -> 데카르트 좌표계 표현(x, y)해 클래스 인스턴스 생성"""
        return cls(r * cos(phi), r * sin(phi))

    @classmethod
    def empty_point(cls):
        """임의의 한 점을 (0, 0)으로 설정해 임시 인스턴스 생성"""
        return cls(0, 0)

    def __add__(self, p2):
        """연산자 '+' 재정의: 두 벡터의 덧셈"""
        return Point(self.x + p2.x, self.y + p2.y)

    def __sub__(self, p2):
        """연산자 '-' 재정의: 두 벡터의 뺄셈"""
        return Point(self.x - p2.x, self.y - p2.y)

    def __mul__(self, c):
        """연산자 '*' 재정의: 벡터의 크기 상수배"""
        return Point(self.x * c, self.y * c)

    def __div__(self, d):
        """연산자 '/' 재정의: 벡터의 크기 상수배"""
        return Point(self.x / d, self.y / d) if d != 0 else Point(0, 0)

    def __eq__(self, p2):
        """연산자 '==' 재정의: 두 벡터(점)가 같은 값인가"""
        return self.x == p2.x and self.y == p2.y

    def dist_from_origin(self):
        """
        (1) 원점으로부터 해당 점까지의 거리
        (2) 벡터의 크기
        """
        return sqrt(pow(self.x, 2.0) + pow(self.y, 2.0))

    def dist_squared_from_origin(self):
        """
        (1) 원점으로부터 해당 점까지의 거리
        (2) 벡터의 크기
        의 제곱
        """
        return pow(self.x, 2.0) + pow(self.y, 2.0)

    def dist_btw_points(self, p2):
        """두 점 간의 거리"""
        return sqrt(pow(self.x - p2.x, 2.0) + pow(self.y - p2.y, 2.0))

    def dist_btw_points2(self, p2_x, p2_y):
        """두 점 간의 거리"""
        return sqrt(pow(self.x - p2_x, 2.0) + pow(self.y - p2_y, 2.0))

    def polar_angle_rad(self):
        """데카르트 좌표계로 표현된 벡터와 x 축까지 각도"""
        return atan2(self.y, self.x)

    def polar_angle_deg(self):
        """데카르트 좌표계로 표현된 벡터와 x 축까지 각도"""
        return degrees(atan2(self.y, self.x))

    def dot(self, p):
        """두 벡터의 내적"""
        return self.x * p.x + self.y * p.y

    def cross(self, p):
        """두 벡터의 곱"""
        return self.x * p.y - self.y * p.x

    def normalized_point(self):
        """단위 벡터 생성"""
        return (self / self.dist_from_origin()) if self.dist_from_origin() > 0.0 else self

    def reflected(self, normal_vec):
        return self - normal_vec * (2 * normal_vec.dot(self))

    def perpendicular(self):
        """원점 대칭"""
        return Point(-self.y, self.x)

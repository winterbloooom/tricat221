#!/usr/bin/env python
# -*- coding:utf-8 -*-

from math import pow, sqrt
# TODO 정리

class Point_Set:
    def __init__(self):
        self.point_set = []
        self.set_size = 0  # len(self.point_size)
        self.begin = None
        self.end = None

    def input_point_set(self, ps):
        self.point_set = ps
        self.set_size = len(ps)
        self.begin = ps[0]
        self.end = ps[-1]

    def append_point(self, p):
        if len(self.point_set) == 0:
            self.begin = p

        self.point_set.append(p)
        self.set_size += 1
        self.end = p
        # self.point_set[-1]로 해도 동일. (self.end)와 p의 아이디(id())가 같음. 같은 포인터

    def projection(self, p):
        a = self.end - self.begin
        b = p - self.begin

        if a.dist_squared_from_origin() != 0:
            projection =  self.begin + a * (a.dot(b) / a.dist_squared_from_origin())
        else:
            projection = self.begin
        return projection

    def dist_to_point(self, p):
        # print("p", p.x)
        # print("원점까지 거리", p.dist_from_origin())
        # print("수선의 발 (", self.projection(p).x, ", ", self.projection(p).y, ")")
        # print("벡터 (", ( p - self.projection(p)).x, ", ", ( p - self.projection(p)).y)
        return (p - self.projection(p)).dist_from_origin()

    def dist_begin_to_end(self):
        return sqrt(pow(self.begin.x - self.end.x, 2) + pow(self.begin.y - self.end.y, 2))

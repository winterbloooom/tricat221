#!/usr/bin/env python
#-*- coding:utf-8 -*-

from math import sqrt, pow


class Point_Set:
    def __init__(self):
        self.point_set = []
        self.set_size = 0 # len(self.point_size)
        self.begin = None
        self.end = None
        self.is_small = False # 개수 몇 개 없어 없는 셈 쳐야 하는 set인가?

    def input_point_set(self, ps): # TODO 작동 확인
        self.point_set = ps
        self.set_size = len(ps)
        self.begin = ps[0]
        self.end = ps[-1]
        self.is_small = False
        # return self

    def append_point(self, p):
        if len(self.point_set)==0:
            self.begin = p # self.point_set[0]

        self.point_set.append(p)
        self.set_size += 1    
        self.end = p #self.point_set[-1]로 해도 동일. (self.end)와 p의 아이디(id())가 같음. 같은 포인터

    def projection(self, p):  # TODO : 이런 연산 함수에서 begin, end 시계방향인가가 영향 주는지 확인!
        a = self.end - self.begin
        b = p - self.begin
        # TODO : divid zero 해결하려 리턴 이렇게 설정함. 점검 필요
        return self.begin + a * (a.dot(b) / a.dist_squared_from_origin()) if a.dist_squared_from_origin() != 0 else self.begin

    def dist_to_point(self, p):
        # print("p", p.x)
        # print("원점까지 거리", p.dist_from_origin())
        # print("수선의 발 (", self.projection(p).x, ", ", self.projection(p).y, ")")
        # print("벡터 (", ( p - self.projection(p)).x, ", ", ( p - self.projection(p)).y)
        return ( p - self.projection(p)).dist_from_origin()

    def dist_begin_to_end(self):
        return sqrt(pow(self.begin.x - self.end.x, 2) + pow(self.begin.y - self.end.y, 2))
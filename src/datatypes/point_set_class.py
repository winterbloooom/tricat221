#!/usr/bin/env python
# -*- coding:utf-8 -*-

from math import pow, sqrt

"""lidar_converter에 사용할 '점의 집합'(그룹) 정의 클래스"""


class PointSet:
    def __init__(self):
        self.point_set = [] # 그룹 내 점의 목록(point_class.py의 Point 클래스)
        self.set_size = 0  # 그룹 내 점의 개수
        self.begin = None # 그룹의 시작점
        self.end = None # 그룹의 끝점

    def input_point_set(self, ps):
        """특정 PointSet의 각 인스턴스를 설정함
        
        Args:
            ps (PointSet): 등록 대상 PointSet
        """
        self.point_set = ps
        self.set_size = len(ps)
        self.begin = ps[0]
        self.end = ps[-1]

    def append_point(self, p):
        """해당 PointSet(그룹)에 한 점을 추가
        
        Args:
            p (Point): 추가할 포인트 개체
        """
        if len(self.point_set) == 0:
            self.begin = p # 그룹의 첫 점이면 begin으로 설정

        self.point_set.append(p) # 점 1개 더 등록
        self.set_size += 1 # 점의 개수 1개 증가
        self.end = p # 추가된 점은 끝점으로 설정. 
        # 위 코드는 self.point_set[-1]로 해도 동일. (self.end)와 p의 아이디(id())가 같음. 같은 포인터

    def projection(self, p):
        """특정 점을 이 PointSet로의 투영점 위치 계산

        PointSet의 첫 점과 끝 점을 선분으로 이은 뒤, 그 위로 점 p의 투영점을 계산함

        Args:
            p (Point): 투영할 포인트 개체
        Returns:
            projection (list): 투영점 좌표
        """
        a = self.end - self.begin # PointSet의 첫 점과 끝 점 이은 벡터
        b = p - self.begin # 첫 점과 특정 점을 이은 벡터

        if a.dist_squared_from_origin() != 0:
            projection = self.begin + a * (a.dot(b) / a.dist_squared_from_origin()) # 내적 이용
        else: # PointSet의 크기가 0 혹은 1인 경우
            projection = self.begin
        
        return projection

    def dist_to_point(self, p):
        """특정 점과 이 PointSet까지의 거리
        
        특정 점의 이 PointSet의 '첫점 ~ 끝점 이은 선분' 위로 내린 수선의 발까지 거리로 계산함

        Args:
            p (Point): 거리를 계산할 포인트
        Returns:
            distance_to_point (float): 점과 그룹 간 거리
        """
        # print("p", p.x)
        # print("원점까지 거리", p.dist_from_origin())
        # print("수선의 발 (", self.projection(p).x, ", ", self.projection(p).y, ")")
        # print("벡터 (", ( p - self.projection(p)).x, ", ", ( p - self.projection(p)).y)
        distance_to_point = (p - self.projection(p)).dist_from_origin()

        return distance_to_point

    def dist_begin_to_end(self):
        """PointSet의 길이: 첫 점과 끝 점까지 거리"""
        return sqrt(pow(self.begin.x - self.end.x, 2) + pow(self.begin.y - self.end.y, 2))

#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
Todo

"""

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def ob_filtering(obstacles, span_angle, angle_range, distance_range):
    """거리, 각도 내 장애물만 걸러 저장"""
    inrange_obstacles = []
    danger_angles = []
    for ob in obstacles:
        # 시작이나 끝점이 안쪽에 있는 것만 inrange로 저장 -> 그렇게까지 잘게 쪼갤 필요는 없음

        # 중앙점만 저장 -> 잘게 쪼개야 함
        begin_ang = int(math.degrees(math.atan2(ob.begin.y, -ob.begin.x))) - span_angle
        end_ang = int(math.degrees(math.atan2(ob.end.y, -ob.end.x))) + span_angle
        middle_x = -(ob.begin.x + ob.end.x) / 2
        middle_y = (ob.begin.y + ob.end.y) / 2
        middle_angle = math.degrees(math.atan2(middle_y, middle_x))
        dist_boat_to_ob = math.sqrt(middle_x**2 + middle_y**2)
        if (angle_range[0] <= middle_angle <= angle_range[1]) and (
            dist_boat_to_ob <= distance_range
        ):
            inrange_obstacles.append(ob)  # [begin.x, begin.y, end.x, end.y]
            danger_angles.extend(list(range(begin_ang, end_ang + 1)))  # 장애물이 있는 각도 리스트
    danger_angles = sorted(list(set(danger_angles)))

    # print("final inrange : ", inrange_obstacles)
    # print("sorted angles: ", danger_angles)

    return inrange_obstacles, danger_angles


def calc_desire_angle(danger_angles, angle_to_goal, angle_range):
    # 선박고정좌표계!
    # 장애물이 있다면 continue

    # 장애물이 없다면 goal, heading까지 각도 계산해 딕셔너리 추가
    # key=angle, value=[to goal, to heading]
    # candidates = {} # candidates[angle] = [abs(psi_goal - angle), abs(psi - angle)]

    # 장애물이 없는데 angle_range 벗어났을 때도 그냥 goal로 가도록 다시 수정함

    psi_desire = 0
    if len(danger_angles) == (angle_range[1] - angle_range[0]) + 1:
        psi_desire = angle_to_goal
    elif len(danger_angles) == 0:
        psi_desire = angle_to_goal
    else:
        delta_goal = 10000
        delta_heading = 10000
        for angle in range(angle_range[0], angle_range[1] + 1):
            if angle in danger_angles:
                continue
            if (delta_goal > abs(angle_to_goal - angle)) or (
                delta_goal == abs(angle_to_goal - angle) and delta_heading > abs(angle)
            ):
                psi_desire = angle
                delta_goal = abs(angle_to_goal - angle)
                delta_heading = abs(angle)
    return psi_desire

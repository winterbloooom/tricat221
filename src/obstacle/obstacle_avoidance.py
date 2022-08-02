#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""get obstacles information and calculate dangerous obstacles and desire angle to change heading

obstacle_avoidance.py

Note:
    * import this file as `import obstacle.obstacle_avoidance as oa`
    * use custum lidar data converter [perception.lidar_converter.py]
"""

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import datatypes.point_class as pc


def ob_filtering(obstacles, dist_to_goal, angle_to_goal, span_angle, angle_range, distance_range):
    """filter dangerous obstacles among all of them

    Args:
        obstacles (ObstacleList) : for one element, obstacle[begin.x, begin.y, end.x, end.y]
        dist_to_goal (float) : distance from boat to goal
        angle_to_goal (float) : relative angle between goal and heading (psi_goal - psi)
        span_angle (int) : adding `span_angle` to extend angle that each obstacles exist
        angle_range (list) : [angle_min, angle_max] to search danger obstacles
        distance_range (float) : distance limit to search danger obstacles

    Returns:
        tuple : (list, list) = (list of danger obstacles(type=Obstacle), list of danger angles(type=int))

    Note:
        * 범위(거리, 각도) 내에 있는 장애물만 걸러내 저장하고, 그 장애물이 위치한 각도도 따로 저장함
        * 단, 목표점이 장애물보다 앞쪽에 있다면(거리가 더 가깝다면)
            * 굳이 그 장애물을 피하지 않고 바로 목표점으로 전진하면 됨
            * 따라서 그 경우에는 저장하지 않음

    Todo:
        * 필터링하는 방식
            * 방법1 : (현재 사용중) 시작이나 끝점이 안쪽에 있는 것만 inrange로 저장 -> lidar converter에서 심하게 잘게 쪼갤 필요는 없음
            * 방법2 : 장애물 중앙점만 사용 -> 잘게 쪼개야 함
        * 목표점과 장애물의 거리 비교 방법
            * 방법1 : 시작/끝/중간 점의 거리만 비교하는 방법
            * 방법2 : 중간 점의 거리만 비교하는 방법
            * 방법3 : 시작과 끝을 연장한 직선을 그리고, 목표점과 현위치(0, 0)이 직선의 각각 양 쪽에 위치하는지(부등식에서 하나는 +, 하나는 -)
            * 방법4 : (현재 사용중) 장애물의 첫점과 끝점을 이은 직선으로 현 보트 위치에서 수선의 발을 내리고, 수선의 발까지 길이(내적 이용)
    """
    inrange_obstacles = []
    danger_angles = []
    for ob in obstacles:
        # 주의! 항상 begin의 각도가 더 작지 않음!!!
        begin_ang = int(math.degrees(math.atan2(ob.begin.y, -ob.begin.x)))
        end_ang = int(math.degrees(math.atan2(ob.end.y, -ob.end.x)))
        if begin_ang > end_ang:
            temp = end_ang
            end_ang = begin_ang
            begin_ang = temp

        begin_ang -= span_angle
        end_ang += span_angle

        middle_x = -(ob.begin.x + ob.end.x) / 2
        middle_y = (ob.begin.y + ob.end.y) / 2
        middle_angle = math.degrees(math.atan2(middle_y, middle_x))

        # dist_to_ob = math.sqrt(middle_x**2 + middle_y**2)
        begin = pc.Point(ob.begin.x, ob.begin.y)
        end = pc.Point(ob.begin.x, ob.begin.y)
        boat = pc.Point(0, 0)
        a = end - begin
        b = boat - begin
        if a.dist_squared_from_origin() != 0:
            projection = begin + a * (a.dot(b) / a.dist_squared_from_origin())
        else:
            projection = begin
        dist_to_ob = (boat - projection).dist_from_origin()

        # 장애물이 목표점보다 뒤에 있어 고려할 필요 없음
        if (begin_ang <= angle_to_goal <= end_ang) and (dist_to_ob >= dist_to_goal):
            print("ob is behind the goal -> don't count")
            continue

        # 장애물이 각도&거리 범위 내에 있음
        if (angle_range[0] <= begin_ang <= angle_range[1]) or (
            angle_range[0] <= end_ang <= angle_range[1]
        ):
            if dist_to_ob <= distance_range:
                inrange_obstacles.append(ob)  # [begin.x, begin.y, end.x, end.y]

                danger_angles.extend(list(range(begin_ang, end_ang + 1)))  # 장애물이 있는 각도 리스트

    # angle_range 넘는 부분은 제외하고 그 안쪽 danger_angles만 남겨둠.
    # 참고 URL: https://appia.tistory.com/101
    danger_angles = set(danger_angles)
    all_angles_in_range = set(range(angle_range[0], angle_range[1] + 1))
    out_of_range = set(danger_angles) - all_angles_in_range
    danger_angles = sorted(list(danger_angles - out_of_range))

    return inrange_obstacles, danger_angles


def calc_desire_angle(danger_angles, angle_to_goal, angle_range):
    """calculate safe and efficient rotation angle considering danger angles

    Args:
        danger_angles (list) : angles which exist inrange_obstacles
        angle_to_goal (float) : distance from boat to goal
        angle_range (list) : [angle_min, angle_max] to search danger obstacles

    Returns:
        float : desire angle(relatvie error angle) which to rotate heading

    Note:
        * 선박고정좌표계임에 유의.
            * safe_angle은 현재 heading(=여기선 0)으로부터 얼마나 돌려야 하는지를 나타냄. (+)가 우회전, (-)가 좌회전
            * 다른 변수들도 heading이 0을 기준으로 함
        * 탐색 범위(=이동 가능 범위) 내 각도를 하나하나 순회하며 안전한 각도를 찾음
            * danger_angles는 장애물이 존재하거나 위험한 각도이므로, 그곳에 해당하는 각도라면 제외
            * goal과 가장 가까운 각도를 찾음
            * goal과 각도차가 같은 여러 개의 각도가 나온다면, heading과 더 가까운 것을 택(덜 움직이는 쪽)
            * 범위 내 장애물이 없거나 범위 내 전부 장애물이 있다면, 범위 벗어나더라도 단순히 goal로 향하도록 함
        * 구현 방법
            * 딕셔너리 이용: key=angle, value=[to goal, to heading]
            * min/max 계산법 이용 (현재 사용 중)
    """
    safe_angle = 0
    if len(danger_angles) == (angle_range[1] - angle_range[0]) + 1:
        print("**** all obs ****")
        return angle_to_goal  # 범위 내 모두 장애물임
    elif len(danger_angles) == 0:
        # inrange_ob가 0이 아닌데 no ob가 되는 경우가 -> 시작점 끝점 대소 관계 재정의해서 해결한 듯
        print("**** no obs ****")
        return angle_to_goal  # 범위 내 장애물 없음
    else:
        delta_goal = 10000  # goal까지 차이각 (절댓값)
        delta_heading = 10000  # heading까지 차이각 (절댓값)
        for angle in range(angle_range[0], angle_range[1] + 1):
            if angle in danger_angles:
                continue  # 장애물이 있는 각도
            if (delta_goal > abs(angle_to_goal - angle)) or (
                delta_goal == abs(angle_to_goal - angle) and delta_heading > abs(angle)
            ):
                safe_angle = angle
                delta_goal = abs(angle_to_goal - angle)
                delta_heading = abs(angle)
    return safe_angle

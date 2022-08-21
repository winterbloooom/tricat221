#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def degree_to_servo(error_angle, angle_alpha, angle_range, servo_range):
    """
    Args:
        error_angle (float): 왼쪽으로 틀어야 하면 -, 오른쪽으로 틀어야 하면 +, 안 움직여도 되면 0
        angle_alpha (float): 민감도 조절 상수. 1 일때 mapping 그대로, 1 초과 시 더 예민하게 움직임
        angle_range (list): [최소, 최대] 각도 범위
        servo_range (list): [최소, 최대] 서보모터 범위

    Note:
                  (x       - input_min        ) * (output_max     - output_min    ) / (input_max         - input_min        ) + output_min
        u_servo = (u_angle - ob_angle_range[0]) * (servo_range[1] - servo_range[0]) / (ob_angle_range[1] - ob_angle_range[0]) + servo_range[0]
    """
    u_angle = (-error_angle) * angle_alpha  # 조절 상수 곱해 감도 조절  # 왼쪽이 더 큰 값을 가져야 하므로

    # degree에서 servo로 mapping
    u_servo = (u_angle - angle_range[0]) * (servo_range[1] - servo_range[0]) / (
        angle_range[1] - angle_range[0]
    ) + servo_range[0]

    # servo motor 제어 가능 범위 내부에 머무르도록 함
    if u_servo > servo_range[1]:
        u_servo = servo_range[1]
    elif u_servo < servo_range[0]:
        u_servo = servo_range[0]

    return int(u_servo)


def pixel_to_degree(target, alpha, angle_range):
    """
    중앙 픽셀에서 얼마나 떨어졌는지 -> 몇 도로 돌려야 하는지
    alpha는 pixel에서 degree 매핑하는 관계 규정. 좀 많이 커야 함 100 이상
    양수면 각도도 양수로 나와 오른쪽으로 가야함

    Args:
        target (list): [area, center_col] 형태의 타겟 표지 정보
        alpha (int): 곱해지는 상수

    Returns:
        float: 스테이션 회피 안할거면 이대로 서보 각으로 변환할 것
    """
    if len(target) == 0:
        # 진입 중 타겟 잃어버림
        return 0
    area = target[0]
    error_pixel = -320 + target[1]
    degree = error_pixel / area * alpha
    if degree > angle_range[1]:
        degree = angle_range[1]
    elif degree < angle_range[0]:
        degree = angle_range[0]
    return degree


def calc_station_vec_end(station_dir, stations):
    """
    Args:
        station_dir : [-180, 180]
        stations : [[station1_x, station1_y], [station2_x, station2_y], [station3_x, station3_y]]

    Returns:
        시점을 스테이션 위치로 하는 단위 벡터의 끝점을 의미함
    """
    n_vec = [
        math.cos(math.radians(station_dir)),
        math.sin(math.radians(station_dir)),
    ]  # 스테이션 방향의 단위벡터
    station_vec_ends = [[0, 0]]  # TODO docking의 waypoints는 도킹 시작 지점도 포함하니까 형상 맞추려고.
    for station in stations:
        vec = [station[0] + n_vec[0], station[1] + n_vec[1]]
        station_vec_ends.append(vec)  # 각 스테이션으로부터의 벡터 끝점

    return station_vec_ends


def project_boat_to_station_vec(stations, station_vec_ends, station_idx, boat):
    # TODO station_idx는 1번 스테이션이 1번임에 유의! 0번은 도킹 시작지점이라 garbage 값
    # 현재 배 위치에서 스테이션 방향으로 투영한 점
    a = [
        station_vec_ends[station_idx][0] - stations[station_idx][0],
        station_vec_ends[station_idx][1] - stations[station_idx][1],
    ]  # [x, y]
    len_a = math.sqrt(a[0] ** 2 + a[1] ** 2)
    b = [boat[0] - stations[station_idx][0], boat[1] - stations[station_idx][1]]
    scala = (a[0] * b[0] + a[1] * b[1]) / len_a
    projection = [stations[station_idx][0] + a[0] * scala, stations[station_idx][1] + a[1] * scala]
    return projection


def follow_station_dir(station_dir, projection, boat, psi, length=1):
    forward_point = [
        projection[0] + length * math.cos(math.radians(station_dir)),
        projection[1] + length * math.sin(math.radians(station_dir)),
    ]
    error_angle = (
        math.degrees(
            math.atan2(
                forward_point[1] - boat[1],
                forward_point[0] - boat[0],
            )
        )
        - psi
    )
    return error_angle, forward_point

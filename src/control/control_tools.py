#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def degree_to_servo(error_angle, angle_alpha, angle_range, servo_range):
    """각도 단위의 회전각을 서보 모터 제어값으로 변경
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
    """픽셀 단위의 값을 각도 단위로 변환

    docking 미션에서 '표지 중앙점 ~ 프레임 중심' 떨어진 픽셀을 선수 회전각으로 변경

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
    """스테이션 앞 지점으로부터 스테이션 방향으로 반직선을 그은 단위 벡터의 끝지점 계산

    Args:
        station_dir (list): [-180, 180] 범위의 스테이션 방향 선수각
        stations (list): 각 스테이션 전방 지점의 좌표

    Returns:
        station_vec_ends (list): 시점을 스테이션 위치로 하는 단위 벡터의 끝점을 의미함
    """
    n_vec = [
        math.cos(math.radians(station_dir)),
        math.sin(math.radians(station_dir)),
    ]  # 스테이션 방향의 단위벡터
    station_vec_ends = [[0, 0]]  # docking의 waypoints는 도킹 시작 지점도 포함하니까 형상 맞추기 위해 [0, 0] 추가
    for station in stations:
        vec = [station[0] + n_vec[0], station[1] + n_vec[1]]
        station_vec_ends.append(vec)  # 각 스테이션으로부터의 벡터 끝점

    return station_vec_ends


def project_boat_to_station_vec(stations, station_vec_ends, station_idx, boat):
    """현재 보트 위치를 도킹할 스테이션의 벡터 위로 투영한 위치

    Args:
        stations (list): 각 스테이션 앞 지점의 위치 좌표 모음
        station_vec_ends (list): 각 스테이션 앞 지점부터 스테이션 방향으로 그은 단위벡터의 끝지점 좌표 모음
        station_idx (int): 도킹해야 하는 스테이션 번호
        boat (list): 현재 보트의 위치

    Returns:
        projection (list): 벡터 위로 투영한 투영점 좌표
    """
    a = [
        station_vec_ends[station_idx][0] - stations[station_idx][0],
        station_vec_ends[station_idx][1] - stations[station_idx][1],
    ]  # 스테이션 앞 지점에서 그은 스테이션 방향 벡터
    len_a = math.sqrt(a[0] ** 2 + a[1] ** 2)  # 벡터 a의 길이
    b = [boat[0] - stations[station_idx][0], boat[1] - stations[station_idx][1]]  # 보트와 스테이션 앞 지점을 이은 벡터
    scala = (a[0] * b[0] + a[1] * b[1]) / len_a  # 벡터 a, b의 내적 계산
    projection = [stations[station_idx][0] + a[0] * scala, stations[station_idx][1] + a[1] * scala]  # 투영점 좌표

    return projection


def follow_station_dir(station_dir, projection, boat, psi, length=1):
    """스테이션 방향의 벡터 위로 추종점 계산

    보트가 벡터를 따라 도킹할 수 있도록 벡터 위로 추종점 설정

    Args:
        station_dir (float): 스테이션 방향의 선수각
        projection (list): 스테이션 방향 벡터 위로 현재 보트 위치의 투영점
        boat (list): 현재 보트 위치 좌표
        psi (float): 현재 보트 선수각
        length (float): 보트의 스테이션 방향 벡터 위로의 투영점으로부터 몇 m 앞의 점을 추종점으로 삼을 것인가

    Returns:
        error_angle (float): 추종점으로 가기 위한 에러각
        forward_point (list): 추종점 위치
    """
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

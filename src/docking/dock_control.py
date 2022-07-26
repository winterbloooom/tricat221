#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""탐지한 타겟으로 도킹

dock.py
===========

Todo
    * 얼마나 틀어야 몇 픽셀인지 결정해야 함
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def dock(target_detected, target, alpha):
    if target_detected:
        error_angle = pixel_to_degree(target, alpha)  # 타겟을 발견했으므로 그를 추종하도록 함
        return error_angle, False
    else:
        error_angle = -999
        return error_angle, True


def avoide_station_collision():
    """스테이션 회피각 리턴"""


def pixel_to_degree(target, alpha):
    """
    중앙 픽셀에서 얼마나 떨어졌는지 -> 몇 도로 돌려야 하는지
    alpha는 pixel에서 degree 매핑하는 관계 규정. 좀 많이 커야 함 100 이상

    Args:
        target (list): [area, center_col] 형태의 타겟 표지 정보
        alpha (int): 곱해지는 상수

    Returns:
        float: 스테이션 회피 안할거면 이대로 서보 각으로 변환할 것

    Todo:
        * alpha 파라미터 이름 수정
    """
    area = target[0]
    error_pixel = 320 - target[1]
    return error_pixel / area * alpha


def degree_to_servo(error_angle, angle_range, servo_range, alpha, use_prev=False):
    """
    Args:
        error_angle (float): 왼쪽으로 틀어야 하면 -, 오른쪽으로 틀어야 하면 +, 안 움직여도 되면 0
        angle_range (list): [angle_min, angle_max]
        servo_range (list): [servo_min, servo_max]
        alpha (int): 조정 상수

    Note:
                  (x       - input_min     ) * (output_max     - output_min    ) / (input_max      - input_min     ) + output_min
        u_servo = (u_angle - angle_range[0]) * (servo_range[1] - servo_range[0]) / (angle_range[1] - angle_range[0]) + servo_range[0]
    """
    if use_prev:
        return -1

    angle_mid = sum(angle_range) / 2
    u_angle = error_angle - angle_mid  # 부호 반대여야 하는데, 서보는 왼쪽이 더 커져야 하니까 이렇게 함
    u_servo = (u_angle - angle_range[0]) * (servo_range[1] - servo_range[0]) / (
        angle_range[1] - angle_range[0]
    ) + servo_range[0]
    return u_servo * alpha
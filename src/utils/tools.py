#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

def rearrange_angle(input_angle):
    if input_angle >= 180:  # 왼쪽으로 회전이 더 이득
        output_angle = -180 + abs(input_angle) % 180
    elif input_angle <= -180:
        output_angle = 180 - abs(input_angle) % 180
    else:
        output_angle = input_angle
    return output_angle
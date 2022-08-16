#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def rearrange_angle(input_angle):
    if input_angle >= 180:  # 왼쪽으로 회전이 더 이득
        output_angle = -180 + abs(input_angle) % 180
    elif input_angle <= -180:
        output_angle = 180 - abs(input_angle) % 180
    else:
        output_angle = input_angle
    return output_angle


def get_log_file_name(mission):
    run_date = time.strftime(
        "%y%m%d", time.localtime(time.time())
    )  # os.popen('date "+%y%m%d"').read()
    run_time = time.strftime(
        "%H%M%S", time.localtime(time.time())
    )  # os.popen('date "+%H%M%S"').read()
    shell_command = (
        "find $(rospack find tricat221)/etc/log/{}-??????-{}-* 2>/dev/null | wc -l".format(
            run_date, mission
        )
    )
    log_num = int(os.popen(shell_command).read()) + 1
    log_file_name = "{}-{}-{}-{:>02d}.log".format(run_date, run_time, mission, log_num)
    file_path = os.popen("rospack find tricat221").read().rstrip() + "/etc/log/" + log_file_name
    return file_path

#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def rearrange_angle(input_angle):
    """if angle is over 180 deg or under -180 deg, rearrange it
    
    Args:
        input_angle (float): angle to rearrange
    Returns:
        output_angle (float): angle after rearrange
    """
    if input_angle >= 180:  # 왼쪽으로 회전이 더 이득
        output_angle = -180 + abs(input_angle) % 180
    elif input_angle <= -180: # 오른쪽으로 회전이 더 이득
        output_angle = 180 - abs(input_angle) % 180
    else:
        output_angle = input_angle
    return output_angle

def moving_avg_filter(queue, queue_size, input, use_prev=False):
    """Moving Average Filter

    Args:
        queue (list): a queue saving the last 'n(= queue_size)' data
        queue_size (int): the size of queue
        input (float): new input data
        use_prev (bool): Don't push input value in queue and just return previous average (default: False)

    Note:
        * The affect of the filter # TODO 사진 올리고 깃헙 링크 첨부
        * Moving Average Filter: 
            https://github.com/winterbloooom/kalman-filter/blob/main/scripts/part1-%EC%9E%AC%EA%B7%80%ED%95%84%ED%84%B0.ipynb
    """
    # Version 1: If "use_prev", do not push new input value and return previous average
    if not use_prev:
        if len(queue) >= queue_size:
            queue.pop(0)
        queue.append(input)
    return sum(queue) / float(len(queue))

    # Version 2: If "use_prev", push last data in queue and return updated average
    # if use_prev:
    #     queue.pop(0)
    #     queue.append(queue[-1])
    # else:
    #     queue.pop(0)
    #     queue.append(input)
    # return sum(queue) / len(queue)


def make_log_file_name(mission):
    """Make log file name. Same file name with bag file

    Args:
        misson (str): docking, auto, or hopping

    Returns:
        file_path (str): path of log file
    """
    run_date = time.strftime("%y%m%d", time.localtime(time.time()))
    run_time = time.strftime("%H%M%S", time.localtime(time.time()))
    shell_command = "find $(rospack find tricat221)/data/log/{}-??????-{}-* 2>/dev/null | wc -l".format(run_date, mission)
    log_num = int(os.popen(shell_command).read()) + 1
    log_file_name = "{}-{}-{}-{:>02d}.log".format(run_date, run_time, mission, log_num)
    file_path = os.popen("rospack find tricat221").read().rstrip() + "/data/log/" + log_file_name
    
    return file_path

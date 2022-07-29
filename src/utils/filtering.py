#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
"""


def moving_avg_filter(queue, queue_size, input, use_prev=False):
    """
    Args:
        queue (list): n 개 크기를 가지는 큐
        input (float): 새로 들어온 데이터
    """
    # 필터에 추가 없이 이전의 평균값을 그대로 사용하는 버전
    if not use_prev:
        if len(queue) >= queue_size:
            queue.pop(0)
        queue.append(input)
    return sum(queue) / len(queue)

    # # 마지막 서보 값을 추가하고 새 평균을 사용하는 버전
    # if use_prev:
    #     queue.pop(0)
    #     queue.append(queue[-1])
    # else:
    #     queue.pop(0)
    #     queue.append(input)
    # return sum(queue) / len(queue)

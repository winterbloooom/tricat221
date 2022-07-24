#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
"""


def moving_avg_filter(queue, input):
    """
    Args:
        queue (list): n 개 크기를 가지는 큐
        input (float): 새로 들어온 데이터
    """

    queue.pop(0)
    queue.append(input)
    return sum(queue) / len(queue)
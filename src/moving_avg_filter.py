#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import csv

csv_file = open("../etc/rosbag/0429_heading.csv", "r", encoding="utf_8")
f = csv.reader(csv_file, delimiter=",", skipinitialspace=True)
print(f)



def MovingAverageFilter(prev_data, n, x):
    """
    (param)     prev_data : 이전 데이터가 저장된 배열
                n : 이동평균필터 큐의 크기
                x : 새로 들어온 데이터
    (return)    avg : 이동평균
    """

    # n개의 데이터가 모이기 전까지는 무시하는 버전
    # if len(prev_data) < n:
    #     return 0

    if len(prev_data) >= n:
        prev_data.pop(0)
    prev_data.append(x)

    return sum(prev_data) / len(prev_data)

# n_samples = 500
# n = 10

# x_filtered = []
# prev_data = []

# x_filtered = [MovingAverageFilter(prev_data, n, x) for x in sonar_data[:n_samples]]

# time = np.arange(0, n_samples)
# plt.plot(time, sonar_data[:n_samples], '.', color='dodgerblue', label='original')
# plt.plot(time, x_filtered, 'r-', label='filtered')

# plt.legend()
# plt.xlabel('Time [sec]')
# plt.ylabel('Altitude [m]')

# plt.savefig('../output/part1-chapter2-moving_average_filter.png')
# plt.show()
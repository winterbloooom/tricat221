#!/usr/bin/env python
#-*- coding:utf-8 -*-

import time
from httplib2 import RelativeURIError
import pandas as pd # pip install pandas
import rospy
import numpy as np

from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

class IMU_MAF:
    def __init__(self):
        rospy.Subscriber("/imu/mag", MagneticField, self.Magnetic_callback, queue_size=1)
        self.pub = rospy.Publisher('f_imu_mag', MagneticField, queue_size=10)

        self.raw_time = []
        self.raw_data = []
        
        self.x_prev = []
        self.y_prev = []
        self.z_prev = []
        # self.record = [["time_sec"], ["time_nsec"], ["raw_x"], ["raw_y"], ["raw_z"], ["f_x"], ["f_y"], ["f_z"]]
        self.record = [[], [] ,[] ,[] ,[] ,[] ,[] ,[]]
        self.n = 10
        self.output_msg = MagneticField()

    def Magnetic_callback(self, msg):
        self.imu_mag = msg

    def filter_data(self):
        self.raw_time = [self.imu_mag.header.stamp.secs, self.imu_mag.header.stamp.nsecs]
        self.record[0].append(self.raw_time[0])
        self.record[1].append(self.raw_time[1])

        magnetic_x = self.imu_mag.magnetic_field.x
        magnetic_y = self.imu_mag.magnetic_field.y
        magnetic_z = self.imu_mag.magnetic_field.z
        self.raw_data = [magnetic_x, magnetic_y, magnetic_z]


        self.record[2].append(magnetic_x)
        self.record[3].append(magnetic_y)
        self.record[4].append(magnetic_z)

        f_x = MovingAverageFilter(self.x_prev, self.n, magnetic_x)
        f_y = MovingAverageFilter(self.y_prev, self.n, magnetic_y)
        f_z = MovingAverageFilter(self.z_prev, self.n, magnetic_z)

        self.record[5].append(f_x)
        self.record[6].append(f_y)
        self.record[7].append(f_z)

        self.output_msg.header.stamp.secs = self.raw_time[0]
        self.output_msg.header.stamp.nsecs = self.raw_time[1]
        self.output_msg.magnetic_field.x = f_x
        self.output_msg.magnetic_field.y = f_y
        self.output_msg.magnetic_field.z = f_z

        self.pub.publish(self.output_msg)

def MovingAverageFilter(prev_data, n, x):
    """
    (param)     prev_data : 이전 데이터가 저장된 배열
                n : 이동평균필터 큐의 크기
                x : 새로 들어온 데이터
    (return)    avg : 이동평균
    """
    if len(prev_data) >= n:
        prev_data.pop(0)
    prev_data.append(x)
    return sum(prev_data) / len(prev_data)

def main():
    rospy.init_node('MAF_IMU', anonymous=True)
    rate = rospy.Rate(10)
    imu_maf = IMU_MAF()
    file_name = '/home/lumos/tricat/src/tricat221/src/' + time.strftime('%Y-%m-%d_%H-%M-%S') + '.csv'
    rospy.sleep(1)

    while not rospy.is_shutdown():
        imu_maf.filter_data()
        rate.sleep()

    ### for saving to csv file
    # result = np.asarray(imu_maf.record)
    # result = np.transpose(result)
    # np.savetxt(file_name, result, delimiter=",")

if __name__ == '__main__':
    main()
#!/usr/bin/env python
#-*- coding:utf-8 -*-

import time
import pandas as pd # pip install pandas

from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

class IMU_MAF:
    def __init__(self) -> None:
        rospy.Subscriber("/imu/mag", MagneticField, self.Magnetic_callback, queue_size=1)
        self.pub = rospy.Publisher('f_imu_mag', MagneticField, queue_size=10)

        self.raw_time = []
        self.raw_data = []
        self.f_data = []
        self.record = [["time_sec"], ["time_nsec"], ["raw_x"], ["raw_y"], ["raw_z"], ["f_x"], ["f_y"], ["f_z"]]
        self.n = 5
        self.output_msg = MagneticField()

    def Magnetic_callback(self, msg):
        self.imu_mag = msg

    def filter_data(self):
        #self.raw_time = [self.imu_mag.header.stamp.sec, self.imu_mag.header.stamp.nsec]
        self.record[0] = self.imu_mag.header.stamp.sec  #self.raw_time[0]
        self.record[1] = self.imu_mag.header.stamp.nsec #self.raw_time[1]

        magnetic_x = self.imu_mag.magnetic_field.x
        magnetic_y = self.imu_mag.magnetic_field.y
        magnetic_z = self.imu_mag.magnetic_field.z
        self.raw_data = [magnetic_x, magnetic_y, magnetic_z]

        for i in range(3):    
            self.f_data[i] = MovingAverageFilter(self.f_data[i], self.n, self.raw_data[i])
            self.record[i+2] = self.raw_data[i]
            self.record[i+5] = self.f_data[i]

        self.output_msg.header.stamp.sec = self.record[0]
        self.output_msg.header.stamp.nsec = self.record[1]
        self.output_msg.magnetic_field.x = self.f_data[0]
        self.output_msg.magnetic_field.y = self.f_data[1]
        self.output_msg.magnetic_field.z = self.f_data[2]

        pub.publish(self.output_msg)

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
    rate = rospy.Rate(5)
    imu_maf = IMU_MAF()
    file_name = './' + time.strftime('%Y-%m-%d_%H-%M-%S') + '.csv'

    while not rospy.is_shutdown():
        imu_maf.filter_data()
        rate.sleep()
    
    df = pd.DataFrame(imu_maf.record)
    df = df.transpose()
    df.to_csv(file_name, header=False, index=False)

if __name__ == '__main__':
    main()
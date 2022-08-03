#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

import rospy
import cv2

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16

import utils.filtering as filtering

class ServoTest:
    def __init__(self):
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.angle_range = rospy.get_param("rotate_angle_range")  # 회전할 각도 범위
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.angle_alpha = rospy.get_param("angle_alpha")  # angle_to_servo 함수에서 사용할 상수
        self.filter_queue = []  # 서보모터값을 필터링할 이동평균필터 큐
        self.error_angle = 0

        cv2.namedWindow("trackbar")
        cv2.createTrackbar('angle_range','trackbar', 80, 90, lambda x : x)
        cv2.createTrackbar('servo_right (low)   [68 or 71]','trackbar', 68, 80, lambda x : x)
        cv2.createTrackbar('servo_left (high) [118 or 121]','trackbar', 118, 130, lambda x : x)
        cv2.createTrackbar('ROTATE HEADING','trackbar', 90, 180, lambda x : x) # - 90 해서 연산


    def degree_to_servo(self, error_angle):
        """
        Args:
            error_angle (float): 왼쪽으로 틀어야 하면 -, 오른쪽으로 틀어야 하면 +, 안 움직여도 되면 0

        Todo:
            * docking과 합쳐서 유틸에서 하기

        Note:
                      (x       - input_min        ) * (output_max     - output_min    ) / (input_max         - input_min        ) + output_min
            u_servo = (u_angle - ob_angle_range[0]) * (servo_range[1] - servo_range[0]) / (ob_angle_range[1] - ob_angle_range[0]) + servo_range[0]
        """
        # angle_mid = sum(self.ob_angle_range) / 2  # 중앙 각도
        # u_angle = angle_mid - error_angle  # 중앙(heading=0으로 두고)으로부터 돌려야 할 각도

        u_angle = -error_angle  # 왼쪽이 더 큰 값을 가져야 하므로

        # degree에서 servo로 mapping
        u_servo = (u_angle - self.angle_range[0]) * (
            self.servo_range[1] - self.servo_range[0]
        ) / (self.angle_range[1] - self.angle_range[0]) + self.servo_range[0]  
        u_servo *= self.angle_alpha  # 조절 상수 곱해 감도 조절

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]
        return int(u_servo)

if __name__=="__main__":
    rospy.init_node("servo_test", anonymous=False)
    st = ServoTest()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) == 27:
            break

        st.angle_range[0] = cv2.getTrackbarPos('angle_range', 'trackbar') * (-1)
        st.angle_range[1] = cv2.getTrackbarPos('angle_range', 'trackbar')
        st.servo_range[0] = cv2.getTrackbarPos('servo_right (low)', 'trackbar')
        st.servo_range[1] = cv2.getTrackbarPos('servo_left (high)', 'trackbar')
        st.error_angle = cv2.getTrackbarPos('ROTATE HEADING', 'trackbar') - 90
        u_servo = st.degree_to_servo(st.error_angle)

        print("-"*50)
        print("angle [{}, {}] -> servo [{}, {}]".format(st.angle_range[0], st.angle_range[1], st.servo_range[0], st.servo_range[1]))
        print("Left --- Mid --- Right")
        print("{:<4}     {:<3}     {:<4}".format(st.servo_range[1], (st.servo_range[0] + st.servo_range[1])/2, st.servo_range[0]))
        print("Angle -> Servo")
        print("{:<5}    {:<5}".format(st.error_angle, u_servo))

        # u_servo_f = filtering.moving_avg_filter(u_servo)
        # print("Angle -> Servo -> Filter")
        # print("{:<5}    {:<5}    {:<6}".format(st.error_angle, u_servo, u_servo_f))

        print("")

        st.servo_pub.publish(u_servo)
        rate.sleep()


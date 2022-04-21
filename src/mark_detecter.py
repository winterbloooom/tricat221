#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


class StarboardCam:
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.starboard_callback)
        
        self.bridge = CvBridge()
        self.img_raw = np.empty(shape=[0])  # TODO 수정할 것
        
        self.kernel_size = 5

        self.p1 = 0 # 좌측 끝단부터 표지 중점까지 길이(픽셀 x좌표 차)
        self.p2 = 0 # 표지 중점부터 중앙 수직선까지 길이(픽셀 x좌표 차)

        target = rospy.get_param("target").split('-')
            # TODO 잘 작동하나 확인
            # TODO yaml 수정 / yello-triangle 식으로
        self.target_color = target[0]
        self.target_shape = target[1]

        cv2.namedWindow("hsv")
        cv2.createTrackbar("H low", "hsv", 0, 180)
        cv2.createTrackbar("H high", "hsv", 90, 180)
        cv2.createTrackbar("S low", "hsv", 0, 255)
        cv2.createTrackbar("S high", "hsv", 122, 255)
        cv2.createTrackbar("V low", "hsv", 0, 255)
        cv2.createTrackbar("V high", "hsv", 122, 255)

    def starboard_callback(self, msg):
        self.img_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_mark(self):  # 한 프레임 당 
        """
        검출되면 T/F 리턴 -> 검출은 됐지만 원하는 비율이 아님 / 검출도 됐고 원하는 비율임 / 검출 안 됐음 으로 나눠 리턴?

        main에서
        while True:
            while not starCam.img_raw.size == (640 * 480 * 3):
                continue

            if starCam.detect_mark():
                # 검출되면 True.
                starCam.mark_pos()
            else:
                # 검출 안 되었으면 계속 전진
        """
        bright_adjust = self.mean_brightness() # 1. 평균 밝기로 변환
        blur = cv2.GaussianBlur(bright_adjust, (self.kernel_size, self.kernel_size), 0) # 2. 가우시안 블러
        mask = self.HSV_mask()
        color_selected = np.empty(shape=[0])  # TODO 수정할 것
        cv2.copyTo(blur, color_selected, mask)
        gray = np.empty(shape=[0])  # TODO 수정할 것
        cv2.cvtColor(color_selected, gray, cv2.COLOR_HSV2GRAY)
        # 이진화 어떻게...? HSV 하고 반전!
        # 모양 검출 -> 플래그 설정할까?
        if # 검출됨:
            self.p1, self.p2 = # 설정하기
            return True
        else:
            self.p1, self.p2 = -1, -1   # 음...?
            return False


    def mean_brightness(self, img):
        pass

    def set_HSV_value(self):

        
    def HSV_mask(self):
        pass

    def mark_pos(self):
        # 음???
        return self.p1, self.p2


# class BowCam:
#     def __init__(self) -> None:
#         self.frame_mid = 0
#         self.mark_mid = 0

#     def mark_check(self):
#         """
#         원하는 마크가 맞는지 체크. 확인 후에는 detection이 아닌 tracking을 중심으로 사용
#         """
#         pass

#     def error_to_middle(self):
#         """
#         표지 중앙과 카메라 중앙이 맞는지 확인 후 에러값 리턴
#         """
#         return (self.frame_mid - self.mark_mid)


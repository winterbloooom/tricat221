#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math

import cv2
import numpy as np
import pymap3d as pm
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64, UInt16


class StarboardCam:
    def __init__(self, target):

        # rospy.Subscriber("/usb_cam/image_raw/", Image, self.starboard_callback)
        rospy.Subscriber("/camera/color/image_raw/", Image, self.starboard_callback)

        self.bridge = CvBridge()
        self.img_raw = np.empty(shape=[0])  # TODO 수정할 것

        self.w = 320
        self.h = 240

        self.gaussian_kernel = 5  # TODO 파라미터화하거나 트랙바로 돌릴 것
        self.morph_kernel = 9  # 5, 7, 9, 13
        self.eps = 0.02
        self.min_area = 500
        self.circ_area_range = [0.5, 1.5]

        # target = rospy.get_param("target").split('-')
        # TODO 잘 작동하나 확인
        # TODO yaml 수정 / yello-모서리 수(3, 4, 5) 식으로
        target = target.split("-")
        self.target_color = target[0]
        self.target_shape = int(target[1])

        cv2.namedWindow("hsv")
        cv2.createTrackbar("H lower", "hsv", 0, 180, self.trackbar_callback)
        cv2.createTrackbar("H upper", "hsv", 13, 180, self.trackbar_callback)
        cv2.createTrackbar("S lower", "hsv", 98, 255, self.trackbar_callback)
        cv2.createTrackbar("S upper", "hsv", 255, 255, self.trackbar_callback)
        cv2.createTrackbar("V lower", "hsv", 0, 255, self.trackbar_callback)
        cv2.createTrackbar("V upper", "hsv", 255, 255, self.trackbar_callback)

        self.H_bound = [0, 0]
        self.S_bound = [0, 0]
        self.V_bound = [0, 0]

        self.bbox = []

    def trackbar_callback(self, usrdata):
        pass

    def starboard_callback(self, msg):
        self.img_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        ### TODO resize하면 연산 좀 줄어드나?
        # self.img_raw = cv2.resize(self.img_raw, (self.h, self.w))

    def find_target_pos(self):  # 한 프레임 당
        while not self.img_raw.size == (640 * 480 * 3):
            return

        detected = False

        # bright_adjust = self.mean_brightness() # 1. 평균 밝기로 변환
        hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV)  # TODO: 여기 맞나?
        blur = cv2.GaussianBlur(hsv, (self.gaussian_kernel, self.gaussian_kernel), 0)  # 2. 가우시안 블러

        self.set_HSV_value()
        mask = self.HSV_mask(blur)

        cv2.imshow("raw", self.img_raw)
        # cv2.imshow("bright_adjust", bright_adjust)
        # cv2.imshow("hsv", hsv)
        # cv2.imshow("blur", blur)

        cv2.imshow("mask", mask)

        ### 모폴로지 연산
        #################3 best 기록
        ### (1, 1) open, H upper 조금 키우기
        ### (3, 3) close, H upper 13
        ### (9, 9) close, 그대로
        ### (5, 5) open, H upper 조금 키우기

        morph_kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)

        _, contours, _ = cv2.findContours(
            morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
        )  # 모드 좋은 걸로 탐색
        # print("# of conturs : {}".format(len(contours)))

        for contour in contours:
            # print("conture size : {}".format(len(contour)))

            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * self.eps, True)

            area = cv2.contourArea(approx)
            if area < self.min_area:
                continue

            vertex_num = len(approx)
            # print("conture size : {}".format(vertex_num))

            if vertex_num == 3:
                shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)

                print("Triangle")

                if self.target_shape == 3:
                    shape = self.set_label(shape, approx, "Triangle")
                    detected = True
            elif vertex_num == 4:
                shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)

                print("Rectangle")

                if self.target_shape == 4:
                    shape = self.set_label(shape, approx, "Rectangle")
                    detected = True
            else:
                _, radius = cv2.minEnclosingCircle(approx)
                ratio = radius * radius * 3.14 / area
                if ratio > self.circ_area_range[0] and ratio < self.circ_area_range[1]:
                    shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)

                    print("Circle")

                    if self.target_shape == 5:
                        shape = self.set_label(shape, approx, "Circle")
                        detected = True

        cv2.imshow("shape", shape)

        if cv2.waitKey(1) == 27:
            return  # 수정

        return True if detected else False

    def set_label(self, img, approx, label):
        drawn = cv2.drawContours(img, [approx], -1, (0, 255, 0), -1)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0))

        self.bbox = [x + w // 2, y + h // 2, w, h]
        print("Bounding Box : [%d, %d, %d, %d]" % (x + w // 2, y + h // 2, w, h))

        return drawn

    def mean_brightness(self):
        img = self.img_raw
        fixed = 70

        m = cv2.mean(img)
        scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
        # TODO 이 방법 맞나...?
        dst = cv2.add(img, scalar)

        return dst

    def set_HSV_value(self):
        self.H_bound[0] = cv2.getTrackbarPos("H lower", "hsv")
        self.H_bound[1] = cv2.getTrackbarPos("H upper", "hsv")
        self.S_bound[0] = cv2.getTrackbarPos("S lower", "hsv")
        self.S_bound[1] = cv2.getTrackbarPos("S upper", "hsv")
        self.V_bound[0] = cv2.getTrackbarPos("V lower", "hsv")
        self.V_bound[1] = cv2.getTrackbarPos("V upper", "hsv")

    def HSV_mask(self, img):
        lower = np.array([self.H_bound[0], self.S_bound[0], self.V_bound[0]])
        upper = np.array([self.H_bound[1], self.S_bound[1], self.V_bound[1]])
        mask = cv2.inRange(img, lower, upper)

        return mask


def main():

    rospy.init_node("Starcam_test", anonymous=True)
    rate = rospy.Rate(10)

    star_cam = StarboardCam("red-4")

    while not rospy.is_shutdown():
        detected = star_cam.find_target_pos()

    rospy.spin()


if __name__ == "__main__":
    main()

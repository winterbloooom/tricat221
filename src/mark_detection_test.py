#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2, sys, time
import numpy as np

class StarboardCam:
    def __init__(self, target):
        # self.img_raw = cv2.imread('C:\coding\\tricat221\\src\\pic23.jpeg', cv2.IMREAD_COLOR)v ~/tricat/src/tricat221/src/pic23.jpeg
        self.img_raw = cv2.imread('pic23.jpeg', cv2.IMREAD_COLOR)
        if self.img_raw is None:
            print("Image load failed!")
            exit(1)
        
        self.w = self.img_raw.shape[0] #640
        self.h = self.img_raw.shape[1] #480

        self.img_raw = cv2.resize(self.img_raw, dsize=(300, 300))

        self.kernel_size = 5

        self.p1 = 0 # 좌측 끝단부터 표지 중점까지 길이(픽셀 x좌표 차)
        self.p2 = 0 # 표지 중점부터 중앙 수직선까지 길이(픽셀 x좌표 차)

        self.target = target #rospy.get_param("target").split('-')
            # TODO 잘 작동하나 확인
            # TODO yaml 수정 / yello-triangle 식으로
            # TODO 나중에는 self. 없애기
        self.target_color = self.target[0]
        self.target_shape = self.target[1]

        cv2.namedWindow("hsv")
        cv2.createTrackbar("H lower", "hsv", 0, 180, self.trackbar_callback)
        cv2.createTrackbar("H upper", "hsv", 16, 180, self.trackbar_callback)
        cv2.createTrackbar("S lower", "hsv", 98, 255, self.trackbar_callback)
        cv2.createTrackbar("S upper", "hsv", 255, 255, self.trackbar_callback)
        cv2.createTrackbar("V lower", "hsv", 0, 255, self.trackbar_callback)
        cv2.createTrackbar("V upper", "hsv", 255, 255, self.trackbar_callback)

        self.H_bound = [0, 0]
        self.S_bound = [0, 0]
        self.V_bound = [0, 0]
    
    def trackbar_callback(self, usrdata):
        pass

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
        color_selected = np.empty(shape=[0])  # TODO 수정할 것


        # bright_adjust = self.mean_brightness() # 1. 평균 밝기로 변환
        hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV) # TODO: 여기 맞나?
        blur = cv2.GaussianBlur(hsv, (self.kernel_size, self.kernel_size), 0) # 2. 가우시안 블러
        
        self.set_HSV_value()
        mask = self.HSV_mask(blur)

        cv2.imshow("raw", self.img_raw)
        # cv2.imshow("bright_adjust", bright_adjust)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("blur", blur)


        color_selected = cv2.bitwise_and(blur, blur, mask=mask)
        # cv2.copyTo(blur, color_selected, mask)
        # # 이진화 어떻게...? HSV 하고 반전!

        cv2.imshow("color_selected", color_selected)

        mask = 255 - mask

        

        # contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # print(len(contours))
        

        # for contour in contours:
        #     print(contour)
        #     approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)
        #     vertex_num = len(approx)

        #     if vertex_num == 3:
        #         self.set_label(mask, contour, "Triangle")
        #     elif vertex_num == 4:
        #         self.set_label(mask, contour, "Rectangle")
        #     else:
        #         area = cv2.contourArea(contour)
        #         _, radius = cv2.minEnclosingCircle(contour)
        #         ratio = radius * radius * 3.14 / area
        #         if ratio > 0.8 and ratio < 1.2:
        #             self.set_label(mask, contour, "Circle")

        
        cv2.imshow("mask", mask)

        # # 모양 검출 -> 플래그 설정할까?
        # if # 검출됨:
        #     self.p1, self.p2 = # 설정하기
        #     return True
        # else:
        #     self.p1, self.p2 = -1, -1   # 음...?
        #     return False
        return False

    def set_label(self, img, pts, label):
        (x, y, w, h) = cv2.boudingRect(pts)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
        print("_________________")
        print(label)

    def mean_brightness(self):
        img = self.img_raw
        fixed = 70

        m = cv2.mean(img)
        scalar = (-int(m[0])+fixed, -int(m[1])+fixed, -int(m[2])+fixed, 0)
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
        # hsv_channels = cv2.split(img)
        # H = hsv_channels[0] #  TODO 순서 맞는지 확인
        # S = hsv_channels[1]
        # V = hsv_channels[2]

        # H = cv2.inRange(H, self.H_bound[0], self.H_bound[1])
        # S = cv2.inRange(S, self.S_bound[0], self.S_bound[1])
        # V = cv2.inRange(V, self.V_bound[0], self.V_bound[1])

        # hsv_img = np.hstack([H, S, V]) # TODO 확인

        # mask = cv2.merge([H, S, V]) # TODO 확인

        lower = np.array([self.H_bound[0], self.S_bound[0], self.V_bound[0]])
        upper = np.array([self.H_bound[1], self.S_bound[1], self.V_bound[1]])
        mask = cv2.inRange(img, lower, upper)

        # cv2.imshow("hsv_img", hsv_img)
        cv2.imshow("mask", mask)

        return mask

if __name__=="__main__":
    starCam = StarboardCam("red-rectangle")

    while True:
        if starCam.detect_mark():
            print("\nDetected")
            print("p1: {} / p2: {}".format(starCam.p1, starCam.p2))
        else:
            print("\nNone")

        time.sleep(60)

        if cv2.waitKey(1) == 27:
            break
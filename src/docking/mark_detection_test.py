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

        self.w = self.img_raw.shape[0] #640로 resize 할 값으로 바꾸기
        self.h = self.img_raw.shape[1] #480

        self.img_raw = cv2.resize(self.img_raw, dsize=(300, 300))

        self.kernel_size = 5

        self.target = target #rospy.get_param("target").split('-')
            # TODO 잘 작동하나 확인
            # TODO yaml 수정 / yello-triangle 식으로
            # TODO 나중에는 self. 없애기
        self.target_color = self.target[0]
        self.target_shape = self.target[1]

        cv2.namedWindow("hsv")
        cv2.createTrackbar("H lower", "hsv", 0, 180, self.trackbar_callback)
        cv2.createTrackbar("H upper", "hsv", 7, 180, self.trackbar_callback)
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
        # bright_adjust = self.mean_brightness() # 1. 평균 밝기로 변환
        hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV) # TODO: 여기 맞나?
        blur = cv2.GaussianBlur(hsv, (self.kernel_size, self.kernel_size), 0) # 2. 가우시안 블러
        
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

        morph_kernel = np.ones((5, 5), np.uint8)
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR) # 임시

        _, contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)  # 모드 좋은 걸로 탐색
        print("# of conturs : {}".format(len(contours)))
        
        for contour in contours:
            # print("conture size : {}".format(len(contour)))

            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)

            area = cv2.contourArea(approx)
            if area < 1000:
                continue


            vertex_num = len(approx)
            # print("conture size : {}".format(vertex_num))

            if vertex_num == 3:
                shape = self.set_label(shape, approx, "Triangle")
            elif vertex_num == 4:
                shape = self.set_label(shape, approx, "Rectangle")
            else:
                # area = cv2.contourArea(approx)  # 여기 0 나와서 div zero 에러 남
                _, radius = cv2.minEnclosingCircle(approx)
                ratio = radius * radius * 3.14 / area
                if ratio > 0.5 and ratio < 1.5: # 조정하자. 트랙바...?
                    self.set_label(shape, approx, "Circle")
            
        cv2.imshow("shape", shape)

        return False


    def set_label(self, img, approx, label):
        drawn = cv2.drawContours(img, [approx], -1, (0, 0, 255), -1)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255))
        return drawn

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
        lower = np.array([self.H_bound[0], self.S_bound[0], self.V_bound[0]])
        upper = np.array([self.H_bound[1], self.S_bound[1], self.V_bound[1]])
        mask = cv2.inRange(img, lower, upper)

        return mask

if __name__=="__main__":
    starCam = StarboardCam("red-rectangle")

    while True:
        if starCam.detect_mark():
            print("\nDetected")
            print("p1: {} / p2: {}".format(starCam.p1, starCam.p2))
        else:
            print("\nNone")

        # time.sleep(60)

        if cv2.waitKey(1) == 27:
            break
#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""camera test

Camera Test for Docking Mission
"""

import cv2
import numpy as np
import rospy
from cv2 import LINE_AA
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraTester:
    def __init__(self):
        rospy.Subscriber("/camera1/usb_cam/image_raw", Image, self.camera_callback)
        rospy.Subscriber("/camera2/usb_cam/image_raw", Image, self.camera_callback2)
        self.bridge = CvBridge()
        self.img_raw = np.empty(shape=(480, 640))  # shape=(height, width)
        self.img_raw2 = np.empty(shape=(480, 640))
        self.mode = rospy.get_param("~mode")

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

    def camera_callback(self, msg):
        self.img_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_callback2(self, msg):
        self.img_raw2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def trackbar_callback(self, usrdata):
        pass

    def detect_mark(self):
        while True:
            # hsv = cv2.resize(cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV), dsize=(0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
            hsv = cv2.resize(
                self.img_raw, dsize=(0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR
            )  # RGB

            self.set_HSV_value()
            mask = self.HSV_mask(hsv)

            cv2.imshow("hsv", mask)
            # cv2.imshow("hsv", hsv)
            cv2.imshow("star", self.img_raw)
            cv2.imshow("bow", self.img_raw2)

            if cv2.waitKey(1) == 27:
                break

    # def show_image(self):
    #     if self.mode=='mark':
    #         self.detect_mark()

    #     while True:
    #         cv2.imshow("img_raw", self.img_raw)
    #         if cv2.waitKey(1) == 27:
    #             return

    def set_HSV_value(self):
        self.H_bound[0] = cv2.getTrackbarPos("H lower", "hsv")
        self.H_bound[1] = cv2.getTrackbarPos("H upper", "hsv")
        self.S_bound[0] = cv2.getTrackbarPos("S lower", "hsv")
        self.S_bound[1] = cv2.getTrackbarPos("S upper", "hsv")
        self.V_bound[0] = cv2.getTrackbarPos("V lower", "hsv")
        self.V_bound[1] = cv2.getTrackbarPos("V upper", "hsv")

    def HSV_mask(self, img):
        hsv_channels = cv2.split(img)
        H = hsv_channels[0]  #  TODO 순서 맞는지 확인
        S = hsv_channels[1]
        V = hsv_channels[2]

        H = cv2.inRange(H, self.H_bound[0], self.H_bound[1])
        S = cv2.inRange(S, self.S_bound[0], self.S_bound[1])
        V = cv2.inRange(V, self.V_bound[0], self.V_bound[1])

        mask = cv2.cvtColor(cv2.merge([H, S, V]), cv2.COLOR_HSV2BGR)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        output_hsv1 = np.concatenate([H, S], axis=1)
        output_hsv2 = np.concatenate([V, mask], axis=1)
        output_hsv = cv2.cvtColor(
            np.concatenate([output_hsv1, output_hsv2], axis=0), cv2.COLOR_GRAY2BGR
        )

        H_text = "H ({} ~ {})".format(self.H_bound[0], self.H_bound[1])
        S_text = "S ({} ~ {})".format(self.S_bound[0], self.S_bound[1])
        V_text = "V ({} ~ {})".format(self.V_bound[0], self.V_bound[1])

        cv2.putText(
            output_hsv,
            H_text,
            (20, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=(0, 0, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            output_hsv,
            S_text,
            (20, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=(0, 0, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            output_hsv,
            V_text,
            (340, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=(0, 0, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            output_hsv,
            "mask",
            (340, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=(0, 0, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )

        return output_hsv


if __name__ == "__main__":
    rospy.init_node("CameraTest", anonymous=True)
    ct = CameraTester()
    rospy.sleep(1)
    ct.detect_mark()

    # ct.show_image()

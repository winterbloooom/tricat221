#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

import numpy as np
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import control.control_tools as control
import dock.mark_detect as mark_detect
import utils.visualizer as visual
from utils.tools import *

# TODO 정리


class Data_Collection:
    def __init__(self):
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gps_fix_callback, queue_size=1)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cam_callback)
        self.bridge = CvBridge()

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        ### 상태
        self.state = 0  # 0, 1, 2
        self.target = []
        self.lat, self.lon = 0, 0

        ### 이미지
        self.raw_img = np.zeros((480, 640, 3), dtype=np.uint8)  # row, col, channel
        self.hsv_img = np.zeros((480, 640), dtype=np.uint8)
        self.shape_img = np.zeros((480, 640, 3), dtype=np.uint8)

        ### 방향
        self.psi = 0  # 자북과 선수 사이 각
        self.psi_desire = 0
        self.station_dir = rospy.get_param("station_dir")
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가

        ### 넓이
        self.mark_area = 0
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.mark_detect_area = rospy.get_param("mark_detect_area")  # 도형이 검출될 최소 넓이
        self.target_detect_area = rospy.get_param("target_detect_area")  # 타겟이라고 인정할 최소 넓이

        ### 색
        self.target_shape = rospy.get_param("target_shape")
        self.target_color = rospy.get_param("target_color")
        self.all_color_ranges = rospy.get_param("color_range")
        self.color_range = np.array(
            [
                [
                    self.all_color_ranges[self.target_color]["color1_lower"],
                    self.all_color_ranges[self.target_color]["color2_lower"],
                    self.all_color_ranges[self.target_color]["color3_lower"],
                ],
                [
                    self.all_color_ranges[self.target_color]["color1_upper"],
                    self.all_color_ranges[self.target_color]["color2_upper"],
                    self.all_color_ranges[self.target_color]["color3_upper"],
                ],
            ]
        )

        ### 시간, 횟수
        ## (state 4에서) '이 시간동안(횟수)' 정지(약한 후진)하고 그 뒤에 헤딩 돌릴 것.
        self.stop_time = rospy.get_param("stop_time")
        ## (state 4에서) 몇 번 정지 신호를 보냈는가?
        self.stop_cnt = 0
        ## (state 5에서) '이만큼(횟수)' 기다리며 얼마나 발견하나 횟수를 셈
        self.target_detect_time = rospy.get_param("target_detect_time")
        ## (state 5에서) 그만큼 중 얼마나 기다렸는가
        self.mark_check_cnt = 0
        ## (state 5에서) 그만큼 기다리는 동안 '얼마나' 발견해야 발견이라 하겠는가
        self.target_detect_cnt = rospy.get_param("target_detect_cnt")
        ## (state 5에서) 그만큼 기다리는 동안 '몇 번' 타겟을 발견했는가
        self.detected_cnt = 0

        ### 최종제어
        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")
        self.thruster_station = rospy.get_param("thruster_station")
        self.thruster_rotate = rospy.get_param("thruster_rotate")
        self.thruster_stop = rospy.get_param("thruster_stop")
        self.thruster_back = rospy.get_param("thruster_back")

        ### 이미지뷰
        cv2.namedWindow("controller")
        cv2.createTrackbar("state", "controller", 1, 2, lambda x: x)
        # cv2.createTrackbar("color", "controller", 0, 2, lambda x: x)
        cv2.createTrackbar("shape", "controller", 0, 2, lambda x: x)
        cv2.createTrackbar("color1 min", "controller", self.color_range[0][0], 180, lambda x: x)
        cv2.createTrackbar("color1 max", "controller", self.color_range[1][0], 180, lambda x: x)
        cv2.createTrackbar("color2 min", "controller", self.color_range[0][1], 255, lambda x: x)
        cv2.createTrackbar("color2 max", "controller", self.color_range[1][1], 255, lambda x: x)
        cv2.createTrackbar("color3 min", "controller", self.color_range[0][2], 255, lambda x: x)
        cv2.createTrackbar("color3 max", "controller", self.color_range[1][2], 255, lambda x: x)
        cv2.createTrackbar("mark_detect_area", "controller", self.mark_detect_area, 3000, lambda x: x)
        cv2.createTrackbar("target_detect_area", "controller", self.target_detect_area, 3000, lambda x: x)
        cv2.createTrackbar("arrival_target_area", "controller", self.arrival_target_area, 8000, lambda x: x)

    def gps_fix_callback(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude

    def heading_callback(self, msg):
        self.psi = msg.data  # [degree]

    def cam_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.size == (640 * 480 * 3):
            self.raw_img = img
        else:
            pass

    def get_trackbar_pos(self):
        """get trackbar poses and set each values"""
        self.state = cv2.getTrackbarPos("state", "controller")
        # self.target_shape = cv2.getTrackbarPos("shape", "controller") + 3
        self.color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
        self.color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
        self.color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
        self.color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
        self.color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
        self.color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
        self.mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller")
        self.target_detect_area = cv2.getTrackbarPos("target_detect_area", "controller")
        self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller")
        # color_pos = cv2.getTrackbarPos("color", "controller")
        # if color_pos == 0:
        #     color = "red"
        # elif color_pos == 1:
        #     color = "green"
        # else:
        #     color = "blue"
        # self.target_color = color
        # self.color_range = np.array(
        #     [
        #         [
        #             self.all_color_ranges[self.target_color]["color1_lower"],
        #             self.all_color_ranges[self.target_color]["color2_lower"],
        #             self.all_color_ranges[self.target_color]["color3_lower"],
        #         ],
        #         [
        #             self.all_color_ranges[self.target_color]["color1_upper"],
        #             self.all_color_ranges[self.target_color]["color2_upper"],
        #             self.all_color_ranges[self.target_color]["color3_upper"],
        #         ],
        #     ]
        # )

    def check_state(self):
        change_state = False
        if self.state == 0:
            # heading 스테이션쪽인지 판단
            change_state = self.check_heading()
        elif self.state == 2:
            if self.mark_check_cnt >= self.target_detect_time:
                change_state = True
                self.mark_check_cnt = 0
                self.detected_cnt = 0
            else:
                change_state = False
        elif self.state == 3:
            # 도킹 완료했는지 확인
            change_state = self.check_docked()

        if change_state:
            print("")
            print("{:=^70}".format(" Change State "))
            print("")
            return True
        else:
            return False

    def check_heading(self):
        # 차잇값이 ref_dir_range 보다 작으면 잘 돌린 것
        angle_to_station = self.station_dir - self.psi
        if angle_to_station >= 180:  # 왼쪽으로 회전이 더 이득
            angle_to_station = -180 + abs(angle_to_station) % 180
        elif angle_to_station <= -180:
            angle_to_station = 180 - abs(angle_to_station) % 180

        return abs(angle_to_station) <= self.ref_dir_range

    def check_target(self, return_target=False):
        self.show_window()
        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True)
        self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링
        target, self.shape_img, self.mark_area = mark_detect.detect_target(
            self.hsv_img,
            self.target_shape,
            self.mark_detect_area,
            self.target_detect_area,
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True

    def check_docked(self):
        if len(self.target) != 0:
            return self.target[0] >= self.arrival_target_area
        else:
            return False

    def print_status(self, error_angle, u_servo, u_thruster):
        print("GPS : lat - {} / lon - {}".format(self.lat, self.lon))
        print("")

        print("State: {}  (0: Rotating Heading\t1: Detecting Target\t2: Docking)".format(self.state))
        print("Shape: {}  (0: Triangle\t\t1: Rectangle\t\t2: Circle)".format(self.target_shape))
        print("")

        if self.state == 0:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

            if u_servo > self.servo_middle:
                servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
            else:
                servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

            if self.stop_cnt >= self.stop_time:
                print("Rotating Heading >>>>")
            else:
                print("Stopping Boat >>>>>>> {:>2d} / {:>2d}".format(self.stop_cnt, self.stop_time))
            print("")
            print("{:^8} - {:^8} = {:^8} {:->9} {:^5}".format("desire", "psi", "error", ">", "servo"))
            print(
                "{:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )

        elif self.state == 1:
            print("Target Shape : {} | Color : {}".format(self.target_shape, self.target_color))
            print("Waiting..... : {:>4d} / {:>4d}".format(self.mark_check_cnt, self.target_detect_time))
            print("Target Cnt   : {:>4d} / {:>4d}".format(self.detected_cnt, self.target_detect_cnt))
            print("")
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )

        elif self.state == 2:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )
            print("")
            print("mid - {:>6} = {:>11} {:->4} {:>11}".format("target", "error_pixel", ">", "error_angle"))
            print(
                "320 - {:>6,.0f} = {:>11,.0f} {:>4} {:>11.2f} {:>9}".format(
                    self.target[1] if len(self.target) != 0 else 0,
                    320 - self.target[1] if len(self.target) != 0 else 0,
                    "",
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")

        print("")
        print("Thruster  : {}".format(u_thruster))
        print("")
        print("-" * 70)

    def show_window(self):
        cv2.moveWindow("controller", 0, 0)
        raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)
        hsv_img = cv2.resize(self.hsv_img, dsize=(0, 0), fx=0.5, fy=0.5)
        hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_GRAY2BGR)
        col1 = np.vstack([raw_img, hsv_img])
        col2 = cv2.resize(self.shape_img, dsize=(0, 0), fx=0.9, fy=1.0)
        show_img = np.hstack([col1, col2])
        cv2.imshow("controller", show_img)

    def visualize(self):
        ids = list(range(0, 10))

        psi_arrow_end_x = 2 * math.cos(math.radians(self.psi))
        psi_arrow_end_y = 2 * math.sin(math.radians(self.psi))
        psi = visual.arrow_rviz(
            name="psi",
            id=ids.pop(),
            x1=0,
            y1=0,
            x2=psi_arrow_end_x,
            y2=psi_arrow_end_y,
            color_r=221,
            color_g=119,
            color_b=252,
        )
        psi_txt = visual.text_rviz(name="psi", id=ids.pop(), text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)

        # psi_desire (가고 싶은 각도)
        desire_arrow_end_x = 3 * math.cos(math.radians(self.psi_desire))
        desire_arrow_end_y = 3 * math.sin(math.radians(self.psi_desire))
        desire = visual.arrow_rviz(
            name="psi_desire",
            id=ids.pop(),
            x1=0,
            y1=0,
            x2=desire_arrow_end_x,
            y2=desire_arrow_end_y,
            color_r=59,
            color_g=139,
            color_b=245,
        )
        desire_txt = visual.text_rviz(
            name="psi_desire",
            id=ids.pop(),
            text="desire",
            x=desire_arrow_end_x,
            y=desire_arrow_end_y,
        )

        axis_x = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[0, 0], [3, 0]],
            color_r=255,
            scale=0.1,
        )
        axis_y = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[0, 0], [0, 3]],
            color_g=255,
            scale=0.1,
        )
        axis_x_txt = visual.text_rviz(name="axis", id=ids.pop(), text="X", x=3.3, y=0)
        axis_y_txt = visual.text_rviz(name="axis", id=ids.pop(), text="Y", x=0, y=3.3)

        min_angle_x = 7 * math.cos(math.radians(self.station_dir - self.ref_dir_range))
        min_angle_y = 7 * math.sin(math.radians(self.station_dir - self.ref_dir_range))
        max_angle_x = 7 * math.cos(math.radians(self.station_dir + self.ref_dir_range))
        max_angle_y = 7 * math.sin(math.radians(self.station_dir + self.ref_dir_range))
        heading_range = visual.linelist_rviz(
            name="heading_range",
            id=ids.pop(),
            lines=[
                [0, 0],
                [min_angle_x, min_angle_y],
                [0, 0],
                [max_angle_x, max_angle_y],
            ],
            color_r=122,
            color_g=114,
            color_b=237,
            scale=0.1,
        )

        all_markers = visual.marker_array_rviz(
            [
                psi,
                psi_txt,
                desire,
                desire_txt,
                axis_x,
                axis_y,
                axis_x_txt,
                axis_y_txt,
                heading_range,
            ]
        )

        return all_markers


def main():
    rospy.init_node("Data_Collect", anonymous=True)
    dc = Data_Collection()
    print_cnt = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        dc.get_trackbar_pos()
        dc.show_window()
        dc.check_state()

        if dc.state == 0:  # 헤딩 돌리기
            if dc.stop_cnt >= dc.stop_time:
                # 정지 종료, 헤딩 돌리기
                u_thruster = dc.thruster_rotate
            else:
                # 아직 멈춰 있어야 함
                dc.stop_cnt += 1
                rate.sleep()
                u_thruster = dc.thruster_back
            error_angle = dc.station_dir - dc.psi
            error_angle = rearrange_angle(error_angle)

        elif dc.state == 1:
            dc.mark_check_cnt += 1
            detected = dc.check_target()
            if detected:
                dc.detected_cnt += 1

            if dc.mark_check_cnt >= dc.target_detect_time:  # 다 기다렸는데
                if dc.detected_cnt >= dc.target_detect_cnt:  # 충분히 많이 검출되면
                    dc.target = dc.check_target(return_target=True)
                    dc.target_found = True  # 타겟 찾은 것
                else:  # 검출 횟수가 적으면
                    dc.target = []
                    dc.target_found = False  # 타겟 못 찾은 것
            else:
                dc.target_found = False

            error_angle = dc.station_dir - dc.psi
            error_angle = rearrange_angle(error_angle)
            u_thruster = dc.thruster_stop

        elif dc.state == 2:  # 스테이션 진입
            dc.target = dc.check_target(return_target=True)
            if len(dc.target) == 0:
                error_angle = error_angle = dc.station_dir - dc.psi
                error_angle = rearrange_angle(error_angle)
            else:
                error_angle = control.pixel_to_degree(dc.target, dc.pixel_alpha, dc.angle_range)  # 양수면 오른쪽으로 가야 함
            u_thruster = dc.thruster_station

        dc.psi_desire = rearrange_angle(dc.psi + error_angle)  # 월드좌표계로 '가야 할 각도'를 계산함

        u_servo = control.degree_to_servo(
            error_angle=error_angle, angle_alpha=dc.angle_alpha, angle_range=dc.angle_range, servo_range=dc.servo_range
        )

        dc.servo_pub.publish(u_servo)
        dc.thruster_pub.publish(u_thruster)

        if print_cnt > 10:
            dc.print_status(error_angle, u_servo, u_thruster)
            print_cnt = 0
        else:
            print_cnt += 1

        all_markers = dc.visualize()
        dc.visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()

# TODO 트랙바에 추가할 것
# 시간 관련
# 도킹 pixel to angle

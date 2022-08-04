#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

import numpy as np
import pymap3d as pm
import rospy

sys.path += os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

import cv2
import dock_control
import mark_detect
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt16

import obstacle.obstacle_avoidance as oa
import perception.gnss_converter as gc
import utils.filtering as filtering
from tricat221.msg import Obstacle, ObstacleList


class Docking:
    def __init__(self):
        # subscribers
        self.heading_sub = rospy.Subscriber(
            "/heading", Float64, self.heading_callback, queue_size=1
        )
        self.enu_pos_sub = rospy.Subscriber(
            "/enu_position", Point, self.boat_position_callback, queue_size=1
        )
        # self.obstacle_sub = rospy.Subscriber(
        #     "/obstacles", ObstacleList, self.obstacle_callback, queue_size=1
        # )
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cam_callback)
        self.bridge = CvBridge()

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        # coordinates
        self.enterence_y, self.enterence_x = gc.enu_convert(rospy.get_param("docking_enterence"))
        self.station1_y, self.station1_x = gc.enu_convert(rospy.get_param("station1"))
        self.station2_y, self.station2_x = gc.enu_convert(rospy.get_param("station2"))
        self.station3_y, self.station3_x = gc.enu_convert(rospy.get_param("station3"))
        self.boat_x, self.boat_y = 0, 0
        self.waypoints = [
            [self.enterence_x, self.enterence_y],
            [self.station1_x, self.station1_y],
            [self.station2_x, self.station2_y],
            [self.station3_x, self.station3_y],
        ]
        self.trajectory = []

        # data
        self.psi = 0  # 자북과 선수 사이 각
        self.raw_img = np.zeros((480, 640, 3), dtype=np.uint8)  # row, col, channel
        # self.hsv_img = np.zeros((480, 640), dtype=np.uint8)
        self.shape_img = np.zeros((480, 640, 3), dtype=np.uint8)

        # ranges, limits
        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.arrival_range = rospy.get_param("arrival_range")  # 도착여부 판단할 범위
        self.color_range = np.array(
            [
                [
                    rospy.get_param("color1_lower"),
                    rospy.get_param("color2_lower"),
                    rospy.get_param("color3_lower"),
                ],
                [
                    rospy.get_param("color1_upper"),
                    rospy.get_param("color2_upper"),
                    rospy.get_param("color3_upper"),
                ],
            ]
        )
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.stop_time = rospy.get_param("stop_time")  # 회전/도착 전후 멈춰서 기다리는 시간
        self.target_detect_time = rospy.get_param("target_detect_time")  # 이 시간동안 발견하길 기다림
        self.target_detect_cnt = rospy.get_param(
            "target_detect_cnt"
        )  # target_detect_time동안 몇 번 발겼했나
        self.station_dir = rospy.get_param("station_dir")

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # ON/OFF
        self.use_pid = rospy.get_param("use_pid")
        self.draw_contour = rospy.get_param("draw_contour")

        # other settings
        self.target_shape = rospy.get_param("target_shape")
        self.thruster_default = rospy.get_param("thruster_default")
        self.thruster_stop = rospy.get_param("thruster_stop")
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.span_angle = rospy.get_param("span_angle")
        self.ob_angle_range = rospy.get_param("ob_angle_range")
        self.ob_dist_range = rospy.get_param("ob_dist_range")

        # current status
        self.state = 5
        # 0: 장애물 회피
        # 1: 스테이션1로 이동 중
        # 2: 스테이션2로 이동 중
        # 3: 스테이션3로 이동 중
        # 4: 헤딩 맞추는 중
        # 5: 타겟 스캔 중
        # 6: 스테이션 진입 중
        # 7: 끝. 정지
        # self.target = {"area": 0, "center_col": 0} # [area, center_col(pixel)] # TODO 딕셔너리로 한꺼번에 바꾸자
        self.target = [0, 0]  # [area, center_col(pixel)]
        self.target_found = False
        self.filter_queue = [0] * self.filter_queue_size
        # cv2.namedWindow("image")

        # controller
        cv2.namedWindow("controller")
        cv2.createTrackbar(
            "color1 lower", "controller", self.color_range[0][0], 180, self.trackbar_callback
        )
        cv2.createTrackbar(
            "color1 upper", "controller", self.color_range[1][0], 180, self.trackbar_callback
        )
        cv2.createTrackbar(
            "color2 lower", "controller", self.color_range[0][1], 255, self.trackbar_callback
        )
        cv2.createTrackbar(
            "color2 upper", "controller", self.color_range[1][1], 255, self.trackbar_callback
        )
        cv2.createTrackbar(
            "color3 lower", "controller", self.color_range[0][2], 255, self.trackbar_callback
        )
        cv2.createTrackbar(
            "color3 upper", "controller", self.color_range[1][2], 255, self.trackbar_callback
        )
        cv2.createTrackbar(
            "ref_dir_range", "controller", self.ref_dir_range, 20, self.trackbar_callback
        )  # 10
        cv2.createTrackbar(
            "arrival_target_area",
            "controller",
            self.arrival_target_area,
            40,
            self.trackbar_callback,
        )  # X 1000하기 / 20
        cv2.createTrackbar(
            "pixel_alpha", "controller", self.pixel_alpha, 10, self.trackbar_callback
        )  # X 100 하기
        cv2.createTrackbar(
            "angle_alpha", "controller", self.angle_alpha, 10, self.trackbar_callback
        )  # 1
        cv2.createTrackbar(
            "filter_queue_size", "controller", self.filter_queue_size, 20, self.trackbar_callback
        )  # 5

    def heading_callback(self, msg):
        self.psi = msg.data  # [degree]

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacles = (
            msg.obstacle
        )  # [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

    def cam_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.size == (640 * 480 * 3):
            self.raw_img = img
        else:
            pass

    def trackbar_callback(self, usrdata):
        pass

    def get_trackbar_pos(self):
        """get trackbar poses and set each values"""
        self.color_range[0][0] = cv2.getTrackbarPos("color1 lower", "controller")
        self.color_range[1][0] = cv2.getTrackbarPos("color1 upper", "controller")
        self.color_range[0][1] = cv2.getTrackbarPos("color2 lower", "controller")
        self.color_range[1][1] = cv2.getTrackbarPos("color2 upper", "controller")
        self.color_range[0][2] = cv2.getTrackbarPos("color3 lower", "controller")
        self.color_range[1][2] = cv2.getTrackbarPos("color3 upper", "controller")
        self.ref_dir_range = cv2.getTrackbarPos("ref_dir_range", "controller")
        self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller") * 1000
        self.pixel_alpha = cv2.getTrackbarPos("pixel_alpha", "controller") * 100
        self.angle_alpha = cv2.getTrackbarPos("angle_alpha", "controller")
        self.filter_queue_size = cv2.getTrackbarPos("filter_queue_size", "controller")

    def is_all_connected(self):
        """make sure all subscribers(nodes) are connected to this node

        Returns:
            bool : True(if all connected) / False(not ALL connected yet)
        """
        not_connected = ""
        if self.heading_sub.get_num_connections() == 0:
            not_connected += "\theadingCalculator"

        if self.enu_pos_sub.get_num_connections() == 0:
            not_connected += "\tgnssConverter"

        # if self.obstacle_sub.get_num_connections() == 0:
        #     not_connected += "\tlidarConverter"

        if not self.raw_img.size == (640 * 480 * 3):
            not_connected += "\tCamera"

        if len(not_connected) == 0:
            return True
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")
            print(not_connected)
            print("\n")
            return False

    def calc_distance(self, point):
        self.distance_to_point = math.hypot(self.boat_x - point[0], self.boat_y - point[1])

        return self.distance_to_point <= self.arrival_range

    def check_state(self):
        change_state = False
        if self.state == 0:
            # 변경지점 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[0])  # TODO 맞는지 확인
        elif self.state == 1:
            # 스테이션1 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[1])  # TODO 맞는지 확인
        elif self.state == 2:
            # 스테이션3 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[2])  # TODO 맞는지 확인
        elif self.state == 3:
            # 스테이션3 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[3])  # TODO 맞는지 확인
        elif self.state == 4:
            # heading 스테이션쪽인지 판단
            change_state = self.check_heading()  # TODO 맞는지 확인
        elif self.state == 5:
            # 마크 발견했는지 확인
            change_state = self.target_found  # TODO 맞는지 확인
        elif self.state == 6:
            # 도킹 완료했는지 확인
            change_state = self.check_docked()  # TODO 맞는지 확인

        if change_state:
            print("=" * 18 + " Change State " + "=" * 18)
            self.state += 1
            return True
        else:
            return False

    def check_heading(self):
        return abs(self.station_dir - self.psi) < self.ref_dir_range

    def check_target(self, return_target=False):
        self.show_window()
        preprocessed = mark_detect.preprocess_image(self.raw_img)
        self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링
        target, self.shape_img = mark_detect.detect_target(
            self.hsv_img, self.target_shape, self.draw_contour
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True  # TODO 작동 확인

    def check_docked(self):
        if len(self.target) != 0:
            return self.target[1] >= self.arrival_target_area
        else:
            return False

    def print_status(self, error_angle, u_servo, u_thruster):
        state_str = [
            "Avoiding Obstacles",
            "Going to Station #1",
            "Going to Station #2",
            "Going to Station #3",
            "Rotating Heading",
            "Detecting Target",
            "Docking",
            "End",
        ]
        # print(abs(self.station_dir - self.psi))
        # if self.target_found:
        #     print(self.target[0])
        print("State: # {} - {}".format(str(self.state), state_str[self.state]))
        print("psi  {:7.2f}  =>  error  {:6.2f} (0 if state #5)".format(self.psi, error_angle))
        if error_angle > 0:
            print("Move     : Right | Servo: {}".format(u_servo))
        elif error_angle < 0:
            print("Move     : Left  | Servo: {}".format(u_servo))
        print("Thruster: {}".format(u_thruster))
        print("-" * 50)

    def show_window(self):
        self.get_trackbar_pos()
        cv2.moveWindow("controller", 0, 0)
        # cv2.moveWindow("image", 1500, 0)
        if self.state in [5, 6]:
            # hsv_img_re = cv2.cvtColor(self.hsv_img, cv2.COLOR_GRAY2BGR)
            show_img = np.hstack([self.raw_img, self.shape_img])
            cv2.imshow("controller", show_img)
        else:
            cv2.imshow("controller", self.raw_img)


def main():
    rospy.init_node("DockingTest", anonymous=True)
    docking = Docking()
    mark_check_cnt = 0
    detected_cnt = 0
    # docking.target_found = False # TODO 이전 값이 잘못 사용되는 경우 없는지 체크

    while not docking.is_all_connected():
        rospy.sleep(0.2)

    print("\n----------All Connected----------\n")

    while not rospy.is_shutdown():
        docking.show_window()
        change_state = docking.check_state()  # TODO 프린트 할 때 바뀌는 지점 알려주기

        if docking.state in [0, 1, 2, 3]:  # TODO 문법 점검
            psi_goal = math.degrees(
                math.atan2(
                    docking.waypoints[docking.state][1] - docking.boat_y,
                    docking.waypoints[docking.state][0] - docking.boat_x,
                )
            )  # 목표까지 떨어진 각도 갱신

        if docking.state == 7:
            # 정지 및 끝내기
            docking.servo_pub.publish(docking.servo_middle)
            docking.thruster_pub.publish(docking.thruster_default)
            print("-" * 20)
            print("Finished!")
            return

        elif docking.state == 0:
            # 시작점으로 이동하며 장애물 회피 # TODO 확인 필
            inrange_obstacles, danger_angles = oa.ob_filtering(
                docking.obstacles,
                docking.distance_to_point,
                psi_goal - docking.psi,
                docking.span_angle,
                docking.ob_angle_range,
                docking.ob_dist_range,
            )  # 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
            error_angle = oa.calc_desire_angle(
                danger_angles, psi_goal - docking.psi, docking.ob_angle_range
            )  # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            # psi_desire = docking.psi + error_angle  # 월드좌표계로 '가야 할 각도'를 계산함

        elif docking.state in [1, 2, 3]:
            # 다음 스테이션으로 이동
            error_angle = psi_goal  # TODO 맞는지 확인
            u_thruster = docking.thruster_default

        elif docking.state == 4:
            # 헤딩 돌리기
            # TODO 정지 명령 몇 초간 내려줘야 하는지 테스트
            error_angle = docking.station_dir - docking.psi  # TODO 잘 작동하나?
            u_thruster = docking.thruster_stop

        elif docking.state == 5:
            # 마크 탐색하기
            if mark_check_cnt > docking.target_detect_time:
                if detected_cnt >= docking.target_detect_cnt:
                    docking.target = docking.check_target(return_target=True)
                    docking.target_found = True
                else:
                    docking.target = [0, 0]
                    docking.target_found = False
                mark_check_cnt = 0
                detected_cnt = 0
            else:
                mark_check_cnt += 1
                detected = docking.check_target()
                if detected:
                    detected_cnt += 1

            error_angle = 0
            u_thruster = docking.thruster_stop

        elif docking.state == 6:
            # 스테이션 진입
            docking.target = docking.check_target(return_target=True)
            error_angle = dock_control.pixel_to_degree(docking.target, docking.pixel_alpha)
            u_thruster = docking.thruster_default

        u_servo = dock_control.degree_to_servo(
            error_angle, docking.angle_range, docking.servo_range, docking.angle_alpha
        )
        u_servo = int(
            filtering.moving_avg_filter(docking.filter_queue, docking.filter_queue_size, u_servo)
        )

        if docking.state == 5:
            u_servo = docking.servo_middle
        docking.servo_pub.publish(u_servo)
        docking.thruster_pub.publish(u_thruster)

        docking.print_status(error_angle, u_servo, u_thruster)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()

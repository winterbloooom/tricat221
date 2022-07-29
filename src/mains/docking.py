#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
Todo
    * 비디오도 시뮬링크 고정
    * 두 개 웹캠 딜레이 해결
    * 파라미터 추가
        hsv? rgb? -> select color의 hsv/rgb도 맞춰주기!
        가우시안 블러 할 건가?
        가우시안 블러 커널 사이즈
        평균밝기 할 건가?
        HSV별로 각각 나타내볼 것인가?
        탐지된 마크 전부 contour 그릴 것인가? 타겟만 그릴 것인가?
"""

import math
import os
import sys

import pymap3d as pm
import rospy

sys.path += os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt16

import docking.dock_control as dock_control
import docking.mark_detect as mark_detect
import docking.obstacle_avoidance as ob_avoid
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
        self.obstacle_sub = rospy.Subscriber(
            "/obstacles", ObstacleList, self.obstacle_callback, queue_size=1
        )
        self.star_cam_sub = rospy.Subscriber("", Image, self.star_cam_callback)  # TODO topic name!
        self.bow_cam_sub = rospy.Subscriber("", Image, self.bow_cam_callback)  # TODO topic name!
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
        self.next_goal = {
            "enterence": [self.enterence_x, self.enterence_y],
            "station1": [self.station1_x, self.station1_y],
            "station2": [self.station2_x, self.station2_y],
            "station3": [self.station3_x, self.station3_y],
            "dock": [0, 0],
        }  # "enterence", "station1", "station2", "station3", "dock"
        # self.next_goal = [[self.enterence_x, self.enterence_y],[self.station1_x, self.station1_y],[self.station2_x, self.station2_y],[self.station3_x, self.station3_y],[0, 0]]

        # =data
        self.psi = 0  # 자북과 선수 사이 각
        self.star_img = None
        self.bow_img = None

        # ranges, limits
        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.arrival_range = rospy.get_param("arrival_range")  # 도착여부 판단할 범위
        self.color_range = [
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
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.stop_time = rospy.get_param("stop_time")  # 회전/도착 전후 멈춰서 기다리는 시간
        self.target_detect_time = rospy.get_param("target_detect_time")  # 이 시간동안 발견하길 기다림
        self.target_detect_cnt = rospy.get_param(
            "target_detect_cnt"
        )  # target_detect_time동안 몇 번 발겼했나
        self.ref_dir = {
            "front": rospy.get_param("ref_front"),
            "side": rospy.get_param("ref_front") + 90,
        }  # reference direction / "front"=도킹스테이션 방향, "side"=front와 90도 이룸
        # TODO 방향 +90 맞나?

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # ON/OFF
        self.use_pid = rospy.get_param("use_pid")
        self.draw_contour = rospy.get_param("draw_contour")

        # other settings
        self.target_shape = rospy.get_param("target_shape")
        self.speed = rospy.get_param("speed")
        self.rate = rospy.Rate(10)  # 10hz, 1초에 10번, 한 번에 0.1초
        self.filter_queue_size = rospy.get_param("filter_queue_size")

        # current status
        self.state = (
            "enterence"  # 0: "enterence", 1: "station1", 2: "station2", 3: "station3", 4: "dock"
        )
        self.target = [0, 0]  # [area, center_col(pixel)]
        self.filter_queue = [0] * self.filter_queue_size

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
            "speed", "controller", self.speed, 20, self.trackbar_callback
        )  # 0 ~ 20 => 1600 ~ 1800 / X 10 + 1600
        cv2.createTrackbar(
            "filter_queue_size", "controller", self.filter_queue_size, 20, self.trackbar_callback
        )  # 5

    def heading_callback(self, msg):
        self.psi = msg.data  # [degree]

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacle = (
            msg.obstacle
        )  # [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

    def star_cam_callback(self, msg):
        self.star_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def bow_cam_callback(self, msg):
        self.bow_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

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
        self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller")
        self.pixel_alpha = cv2.getTrackbarPos("pixel_alpha", "controller")
        self.angle_alpha = cv2.getTrackbarPos("angle_alpha", "controller")
        self.speed = cv2.getTrackbarPos("speed", "controller")
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

        if self.obstacle_sub.get_num_connections() == 0:
            not_connected += "\tlidarConverter"

        if not self.star_img.size == (640 * 480 * 3):
            not_connected += "\tstarCam"

        if not self.bow_img.size == (640 * 480 * 3):
            not_connected += "\tbowCam"

        if len(not_connected) == 0:
            return True
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")
            print(not_connected)
            print("\n")
            return False

    def calc_distance_to_goal(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        goal = self.next_goal[self.state]
        self.distance_to_goal = math.hypot(self.boat_x - goal[0], self.boat_y - goal[1])

        return self.distance_to_goal <= self.arrival_range

    def check_state(self):
        """check whether boat arrive to the goal and change the state

        Returns:
            bool : checked / 정상 상태, 타겟 발견 등
            bool : docked / docking 모드에서 도킹 끝났는가

        Note:
            * "dock"의 경우 goal이 없음. 도착은 타겟과의 거리로 계산함
            * 각 station으로 도착을 했다면, 마크를 탐지하는 과정을 거침(check_station)
                * target을 탐지했다면, 바로 "dock" 모드로 바꿈. 그렇지 않다면 다음 station으로 모드 바꿈
                * "dock" 모드인데 아직 도킹이 완료되지 않았을 때도 실행.
            * 모든 station에서 발견을 못 했다면 다시 1번으로 돌아감

        Todo:
            * "dock"모드여서 전면으로 돌렸는데 거기서 발견 못 했을 때의 처리
            * "dock"모드에서 표지로 점점 가까이 다가갈수록 표지가 화면 밖으로 나가며 영역이 작아지거나 모양이 바뀔 수도 있음!
                * 차라리 몇 초간 마크 판별 없이 tracking 할까
        """
        checked = False  # 도착을 했는가
        docked = False  # 도킹이 다 됐는가

        # 도착 여부 계산
        if self.state == "dock":
            # target 마크의 넓이가 특정 값 이상이면 가깝다고, 즉 도착했다고 판단함
            if self.target[0] >= self.arrival_target_area:
                docked = True
                return checked, docked
        else:
            if not self.calc_distance_to_goal():  # 도착을 안했음
                checked = False
                return checked, docked

        # 도착을 했고, state를 바꿈
        checked = True
        if self.state == "enterence":
            self.state == "station1"
        elif self.state == "station1":
            if self.check_station("side"):  # 타겟 발견
                self.state = "dock"
            else:
                self.state = "station2"
        elif self.state == "station2":
            if self.check_station("side"):  # 타겟 발견
                self.state = "dock"
            else:
                self.state = "station3"
        elif self.state == "station3":
            if self.check_station("side"):  # 타겟 발견
                self.state = "dock"
            else:
                self.state = "station1"  # 모든 스테이션에서 발견 못함 -> 다시 1번으로
        elif self.state == "dock":
            if self.check_station("front"):
                pass  # 돌려서 발견함
            else:
                checked = False  # 돌렸는데 발견 못함

        return checked, docked

    def check_station(self, dir):
        """check whether target is exist in that station

        Args:
            dir (str) : direction to change heading. "front" (bow to station) / "side" (starboard to station)

        Returns:
            bool : True(target detected in this station) / False(that is not the target or any marks detected)

        Note:
            * 1. 일단 정지
            * 2. 선수 방향을 돌림
                * station1 ~ 3 모드에서는 측면으로, dock 모드에서는 정면으로
                * station2, 3에서는 도착 시에도 측면을 바라보고는 있으나, 이동 중에 삐뚤어질 수도 있으니 다시 확인함
            * 3. 타겟인지 마크 확인
        """
        # 1. 일단 정지
        for _ in range(self.stop_time):
            self.thruster_pub.publish(1500)
            self.rate.sleep()

        # 2. 선수 방향 전환
        self.rotate_heading(dir)

        # 3. 타겟 여부 확인
        is_target = self.check_target(dir)

        return is_target

    def rotate_heading(self, dir):
        """rotate heading to specific direction
        dir (str) : direction to change heading. "front" (bow to station) / "side" (starboard to station)

        Note:
            * 1. heading 확인: 현재 선수각이 원하는 각도 범위 이내인가?
            * 2. 원하는 각이 아니라면 차이각만큼 error_angle로 정하고 servo를 작동시킴
            * 3. 원하는 각이 나올 때까지 반복
        """
        while abs(self.ref_dir[dir] - self.psi) > self.ref_dir_range:
            error_angle = self.ref_dir[dir] - self.psi
            u_servo = dock_control.degree_to_servo(
                error_angle, self.angle_range, self.servo_range, self.angle_alpha
            )
            self.servo_pub(u_servo)

    def check_target(self, dir="side"):
        """check whethere target exists in the current location

        Args:
            dir (str) : which camera is used(heading direction). "front" (bow to station) / "side" (starboard to station)

        Returns:
            bool : True(target detected) / False(target not detected)

        Notes:
            * 일정 시간동안 대기하며 몇 번 타겟이 검출되는지 세어봄. 특정 횟수 이상이면 그곳에 타겟이 있다고 판단함
            * "side" 방향일 때, 즉 측면 카메라는 있다/없다만 보기 때문에 self.target에 값을 할당하지 않음.
            * "front" 방향일 때, 즉 정면 카메라는 계속 그 값을 이용해 추적해야 하므로 self.target에 탐지한 정보를 할당함

        Todo:
            * 프레임 중앙 부분에 타겟이 나와야 True로 추가
        """
        target_found = 0
        for _ in range(self.target_detect_time):
            preprocessed = mark_detect.preprocess_image(self.star_img)  # 영상 전처리
            color_picked = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링
            target = mark_detect.detect_target(
                color_picked, self.target_shape, self.draw_contour
            )  # target = [area, center_col] 형태로 타겟의 정보를 받음
            if len(target) == 0:
                target_found += 1  # 타겟이 검출되었다면 검출 횟수 +1
            self.rate.sleep()

        if target_found > self.target_detect_cnt:
            if dir == "front":  # self.state == "dock"과 같은 의미
                self.target = target  # 타겟 정보 저장
            return True
        else:
            return False

    def calc_error_angle_to_goal(self):
        """calculate error angle to go to goal

        Returns:
            float : error angle. (-) to left, (+) to right

        Todo:
            * 아직 못 만듦!
        """
        pass


def main():
    rospy.init_node("Docking", anonymous=True)
    docking = Docking()

    while not docking.is_all_connected():
        rospy.sleep(0.2)

    print("\n----------All Connected----------\n")

    while not rospy.rospy.is_shutdown():
        cv2.moveWindow("controller", 0, 0)
        docking.get_trackbar_pos()

        checked, docked = docking.check_state()  # 모드 바뀜

        if docking.state == "enterence":
            # 장애물 회피 계속하면서 시작점으로 이동하기
            error_angle = ob_avoid.calc_error_angle()
        elif docking.state in ["station1", "station2", "station3"]:
            # 다음 station으로 가야 함
            error_angle = docking.calc_error_angle_to_goal()
        elif docking.state == "dock":
            # if docked:
            #     docking.thruster_pub(1500)  # stop
            #     break
            # else: #if checked:
            #     error_angle = dock_control.pixel_to_degree(docking.target, docking.pixel_alpha)
            error_angle, use_prev_servo = dock_control.dock(
                checked, docking.target, docking.pixel_alpha
            )

        u_servo = dock_control.degree_to_servo(
            error_angle,
            docking.angle_range,
            docking.servo_range,
            docking.angle_alpha,
            use_prev_servo,
        )
        u_servo = filtering.moving_avg_filter(docking.filter_queue, docking.filter_queue_size, u_servo, use_prev_servo)
        if docking.use_pid:
            pass

        docking.servo_pub(u_servo)
        docking.thruster_pub(docking.speed)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break

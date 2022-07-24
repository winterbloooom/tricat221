#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
Todo
    * 비디오도 시뮬링크 고정
    * 두 개 웹캠 딜레이 해결

파라미터

hsv? rgb? -> select color의 hsv/rgb도 맞춰주기!
가우시안 블러 할 건가?
가우시안 블러 커널 사이즈
평균밝기 할 건가?
색공간 초기 range 설정값

HSV별로 각각 나타내볼 것인가?
탐지된 마크 전부 contour 그릴 것인가? 타겟만 그릴 것인가?

pixel -> degree 상수 몇으로?
servo와 angle의 range 각각 어떤 범위로?
degree -> servo 상수 몇으로 할 것인가?
필터 큐 사이즈 몇으로 할 것인가?
PID 쓸 것인가?
"""

import rospy
import math
import pymap3d as pm

import sys, os
sys.path += os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from tricat221.msg import Obstacle, ObstacleList

import perception.gnss_converter as gc
import docking.dock_control as dock_control
import docking.mark_detect as mark_detect
import docking.obstacle_avoidance as ob_avoid


class Docking:
    def __init__(self):
        # subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.star_cam_sub = rospy.Subscriber("", Image, self.star_cam_callback) # TODO topic name!
        self.bow_cam_sub = rospy.Subscriber("", Image, self.bow_cam_callback) # TODO topic name!
        self.bridge = CvBridge()

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0) # TODO 아두이노 쪽에서 S 수정하기
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        # coordinates
        self.enterence_y, self.enterence_x = gc.enu_convert(rospy.get_param("docking_enterence"))
        self.station1_y, self.station1_x = gc.enu_convert(rospy.get_param("station1"))
        self.station2_y, self.station2_x = gc.enu_convert(rospy.get_param("station2"))
        self.station3_y, self.station3_x = gc.enu_convert(rospy.get_param("station3"))
        self.boat_x, self.boat_y = 0, 0
        self.next_goal = {"enterence" : [self.enterence_x, self.enterence_y],
                        "station1" : [self.station1_x, self.station1_y],
                        "station2" : [self.station2_x, self.station2_y],
                        "station3" : [self.station3_x, self.station3_y],
                        "dock" : [0, 0]
                        }   #"enterence", "station1", "station2", "station3", "dock" # TODO dock 골 지점

        # directions
        self.psi = 0 # 자북과 선수 사이 각
        self.ref_dir = {"front" : rospy.get_param("ref_front"), "side" : rospy.get_param("ref_front") + 90} # reference direction / "front"=도킹스테이션 방향, "side"=front와 90도 이룸
            # TODO 방향 +90 맞나?

        # input data
        self.star_img = None
        self.bow_img = None

        # ranges
        self.angle_range = rospy.get_param("angle_range") # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range") # 배열! [min, max]
        self.arrival_range = rospy.get_param("arrival_range") # 도착여부 판단할 범위
        self.color_range = [[0, 0, 0], [0, 0, 0]] # TODO 트랙바 만들어 할당!
        self.ref_dir_range = rospy.get_param("ref_dir_range") # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # status
        self.state = "enterence" #"enterence", "station1", "station2", "station3", "dock"

        # etc
        self.target_shape = rospy.get_param("target_shape")
        self.target = [0, 0]

    def heading_callback(self, msg):
        self.psi = msg.data # [degree]

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacle = msg.obstacle #[msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

    def star_cam_callback(self, msg):
        self.star_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def bow_cam_callback(self, msg):
        self.bow_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def trackbar_callback(self, usrdata):
        pass

    def is_all_connected(self):
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

        if len(not_connected)==0:
            return True
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")
            print(not_connected)
            print("\n")
            return False

    def calc_distance_to_goal(self):
        """
        Returns:
            bool : 도착했으면 True
        """
        goal = self.next_goal[self.state]
        self.distance_to_goal = math.hypot(self.boat_x - goal[0], self.boat_y - goal[1])

        return self.distance_to_goal <= self.arrival_range
    # def arrival_check(self):
    #     self.calc_distance_to_goal() #목적지까지 거리 다시 계산
    #     if self.distance_to_goal <= self.goal_range:
    #         return True
    #     else:
    #         return False

    def check_state(self):
        """
        도착을 했을 때 state 별 행동
        Returns:
            bool : checked / 정상 상태, 타겟 발견 등
            bool : docked / docking 모드에서 도킹 끝났는가
        """
        checked = False
        docked = False

        # goal까지 거리 계산: main으로 빼도 됨
        if self.state == "dock":
            if self.target[0] >= self.arrival_target_area:
                # TODO 점점 가까이 다가갈수록 표지가 화면 밖으로 나가며 영역이 작아지거나 모양이 바뀔 수도 있음! 차라리 몇 초간 마크 판별 없이 tracking 할까
                docked = True
                return checked, docked
        else:
            if not self.calc_distance_to_goal(): # 도착을 안했음
                checked = False
                return checked, docked

        checked = True
        # 도착을 했음
        if self.state == "enterence":
            self.state == "station1"
        elif self.state == "station1":
            if self.check_station():
                self.state = "dock"
            else:
                self.state = "station2"
        elif self.state == "station2":
            if self.check_station():
                self.state = "dock"
            else:
                self.state = "station3"
        elif self.state == "station3":
            if self.check_station():
                self.state = "dock"
            else:
                # TODO 모든 스테이션에서 발견 못함
                self.state = "station1"
        elif self.state == "dock":
            if self.check_station():
                # 돌려서 발견함
                pass
            else:
                # TODO 돌렸는데 발견 못함
                checked = False

        return checked, docked


    def check_station(self, dir):
        """
        # 일단 정지
        # 방향 측면으로
        # 마크 확인
        """
        for _ in range(3):
            self.thruster_pub.publish(1500) # 정지
            rospy.sleep(0.5) # TODO 멈추는 시간은 다시 고려
        
        self.rotate_heading(dir) # 첫 번째 스테이션 아니더라도 삐뚤어질 수 있으니 일단 둠
        is_target = self.check_target(dir)
        return is_target

    def rotate_heading(self, dir):
        """
        dir (str) : front, side

        1. heading 확인
        2. 원하는 각인가? 그렇다면 끝내고 아니라면 반복
        """
        while abs(self.ref_dir[dir] - self.psi) > self.ref_dir_range:
            error_angle = self.ref_dir[dir] - self.psi
            u_servo = dock_control.degree_to_servo(error_angle, self.angle_range, self.servo_range, self.angle_alpha)
            self.servo_pub(u_servo)


    def check_target(self, dir="side"):
        target_found = 0
        for _ in range(90): # TODO 숫자 조정 필요
            preprocessed = mark_detect.preprocess_image(self.star_img)
            color_picked = mark_detect.select_color(preprocessed, self.color_range)
            target = mark_detect.detect_target(color_picked, self.target_shape) #[area, center_col]
            if target is not None:
                target_found += 1
        if target_found > 40:# TODO 숫자 조정 필요
            if dir == "front":
                self.target = target
            return True
        else:
            return False

    def calc_error_angle2222(self):
        pass



def main():
    rospy.init_node('Docking', anonymous=True)
    docking = Docking()

    while not docking.is_all_connected():
        rospy.sleep(0.2)

    print("\n----------All Connected----------\n")

    while not rospy.rospy.is_shutdown():
        checked, docked = docking.check_state() # 모드 바뀜

        if docking.state == "enterence":
            # 장애물 회피 계속하면서 시작점으로 이동하기
            error_angle = ob_avoid.calc_error_angle()
            
        elif docking.state in ["station1", "station2", "station3"]:
            # station1으로 가야 함
            error_angle = docking.calc_error_angle2222()
        elif docking.state == "dock":
            if docked:
                docking.thruster_pub(1500) # stop
                break
            if checked:
                error_angle = dock_control.pixel_to_degree(docking.target, docking.pixel_alpha)

        u_servo = dock_control.degree_to_servo(error_angle, docking.angle_range, docking.servo_range, docking.angle_alpha)
        docking.servo_pub(u_servo)
        docking.thruster_pub(1600) # 파라미터로
        
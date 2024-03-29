#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""2021년 i-Tricat211팀 자율운항 코드 수정본"""

import math
import os
import sys

import numpy as np
import rospy
import skfuzzy as fuzz
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from skfuzzy import control as ctrl
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.gnss_converter as gc
import utils.visualizer as visual
from utils.tools import *


class Fuzzy:
    def __init__(self):
        self.boat_x, self.boat_y = 0, 0
        self.goal_x, self.goal_y, _ = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.trajectory = []  # 지금껏 이동한 궤적

        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        self.distance_to_goal = 100000  # 배~목적지 거리. max 연산이므로 큰 값을 초기 할당
        self.goal_range = rospy.get_param("goal_range")  # 도착이라 판단할 거리(반지름)

        self.psi = 0.0
        self.psi_goal = 0

        self.yaw_rate = 0

        ## LiDAR
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.ranges = []
        self.danger_ob = {}

        ## FUZZY
        self.fuzzy_servo_control = 0
        self.target_servo_ang = None

        ## PID
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2

        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster = rospy.get_param("thruster")

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        self.print_cnt = 0

    def is_all_connected(self):
        not_connected = ""  # 아직 연결되지 않은 센서 목록
        if self.yaw_rate_sub.get_num_connections() == 0:
            not_connected += "IMU\t"
        if self.heading_sub.get_num_connections() == 0:
            not_connected += "headingCalculator\t"
        if self.enu_pos_sub.get_num_connections() == 0:
            not_connected += "gnssConverter\t"
        if self.lidar_sub.get_num_connections() == 0:
            not_connected += "lidar\t"

        if len(not_connected) == 0:
            return True  # 전부 연결됨
        else:
            print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            print(not_connected)
            print("\n")
            return False  # 아직 다 연결되지는 않음

    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self, msg):
        self.psi = msg.data

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def lidar_callback(self, data):
        # print(data.header.seq)

        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges  # list

    def calc_psi_goal(self):
        self.psi_goal = math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)) - self.psi

    def arrival_check(self):
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        return self.distance_to_goal <= self.goal_range

    def servo_pid_controller(self):
        error_angle = self.psi_goal  # P ctrl
        cp_servo = self.kp_servo * error_angle

        yaw_rate = math.degrees(self.yaw_rate)  # D ctrl
        cd_servo = self.kd_servo * (-yaw_rate)

        servo_pd = -(cp_servo + cd_servo)
        u_servo = self.servo_middle + servo_pd

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[1]

        return int(u_servo)

    def fuzzy(self):
        distance = ctrl.Antecedent(np.arange(0, 4, 0.1), "distance")
        angle = ctrl.Antecedent(np.arange(-50, 50, 1), "angle")
        target_servo = ctrl.Consequent(np.arange(-30, 30, 1), "target_servo")

        distance["ED"] = fuzz.trapmf(distance.universe, [0, 0, 1, 2])
        distance["D"] = fuzz.trimf(distance.universe, [1, 2, 3])
        distance["W"] = fuzz.trimf(distance.universe, [2, 3, 4])
        distance["B"] = fuzz.trimf(distance.universe, [3, 4, 4])

        angle["NL"] = fuzz.trapmf(angle.universe, [-70, -40, -30, -20])
        angle["NM"] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10])
        angle["NS"] = fuzz.trimf(angle.universe, [-25, 0, 1])
        angle["PS"] = fuzz.trimf(angle.universe, [0, 1, 25])
        angle["PM"] = fuzz.trimf(angle.universe, [15, 25, 40])
        angle["PL"] = fuzz.trimf(angle.universe, [30, 40, 70])

        target_servo["RRRR"] = fuzz.trimf(target_servo.universe, [-30, -27, -18])
        target_servo["RRR"] = fuzz.trimf(target_servo.universe, [-18, -16, -13])
        target_servo["RR"] = fuzz.trimf(target_servo.universe, [-18, -13, -7])
        target_servo["R"] = fuzz.trimf(target_servo.universe, [-12, -7, 0])
        target_servo["N"] = fuzz.trimf(target_servo.universe, [0, 0, 0])
        target_servo["L"] = fuzz.trimf(target_servo.universe, [0, 7, 12])
        target_servo["LL"] = fuzz.trimf(target_servo.universe, [7, 13, 18])
        target_servo["LLL"] = fuzz.trimf(target_servo.universe, [13, 16, 18])
        target_servo["LLLL"] = fuzz.trimf(target_servo.universe, [18, 27, 30])

        rule_ED_NL = ctrl.Rule(distance["ED"] & angle["NL"], target_servo["RR"])
        rule_ED_NM = ctrl.Rule(distance["ED"] & angle["NM"], target_servo["RRR"])
        rule_ED_NS = ctrl.Rule(distance["ED"] & angle["NS"], target_servo["RRRR"])
        rule_ED_PS = ctrl.Rule(distance["ED"] & angle["PS"], target_servo["LLLL"])
        rule_ED_PM = ctrl.Rule(distance["ED"] & angle["PM"], target_servo["LLL"])
        rule_ED_PL = ctrl.Rule(distance["ED"] & angle["PL"], target_servo["LL"])

        rule_D_NL = ctrl.Rule(distance["D"] & angle["NL"], target_servo["R"])
        rule_D_NM = ctrl.Rule(distance["D"] & angle["NM"], target_servo["RR"])
        rule_D_NS = ctrl.Rule(distance["D"] & angle["NS"], target_servo["RRR"])
        rule_D_PS = ctrl.Rule(distance["D"] & angle["PS"], target_servo["LLL"])
        rule_D_PM = ctrl.Rule(distance["D"] & angle["PM"], target_servo["LL"])
        rule_D_PL = ctrl.Rule(distance["D"] & angle["PL"], target_servo["L"])

        rule_W_NL = ctrl.Rule(distance["W"] & angle["NL"], target_servo["N"])
        rule_W_NM = ctrl.Rule(distance["W"] & angle["NM"], target_servo["R"])
        rule_W_NS = ctrl.Rule(distance["W"] & angle["NS"], target_servo["RR"])
        rule_W_PS = ctrl.Rule(distance["W"] & angle["PS"], target_servo["LL"])
        rule_W_PM = ctrl.Rule(distance["W"] & angle["PM"], target_servo["L"])
        rule_W_PL = ctrl.Rule(distance["W"] & angle["PL"], target_servo["N"])

        rule_B_NL = ctrl.Rule(distance["B"] & angle["NL"], target_servo["N"])
        rule_B_NM = ctrl.Rule(distance["B"] & angle["NM"], target_servo["N"])
        rule_B_NS = ctrl.Rule(distance["B"] & angle["NS"], target_servo["R"])
        rule_B_PS = ctrl.Rule(distance["B"] & angle["PS"], target_servo["L"])
        rule_B_PM = ctrl.Rule(distance["B"] & angle["PM"], target_servo["N"])
        rule_B_PL = ctrl.Rule(distance["B"] & angle["PL"], target_servo["N"])

        target_servo_ctrl = ctrl.ControlSystem(
            [
                rule_ED_NL,
                rule_ED_NM,
                rule_ED_NS,
                rule_ED_PL,
                rule_ED_PM,
                rule_ED_PS,
                rule_D_NL,
                rule_D_NM,
                rule_D_NS,
                rule_D_PL,
                rule_D_PM,
                rule_D_PS,
                rule_W_NL,
                rule_W_NM,
                rule_W_NS,
                rule_W_PL,
                rule_W_PM,
                rule_W_PS,
                rule_B_NL,
                rule_B_NM,
                rule_B_NS,
                rule_B_PL,
                rule_B_PM,
                rule_B_PS,
            ]
        )
        self.target_servo_ang = ctrl.ControlSystemSimulation(target_servo_ctrl)

    def fuzzy_control_avoidance(self):
        """
        Returns:
            bool: 장애물이 탐지 범위(각, 거리) 안에 있으면 True이며, LPP라는 뜻. 그 외엔 False로 GPP
        """
        # 라이다 콜백 함수와 겹쳐서 현재 탐색 중인 데이터가 아닌 방금 들어온 데이터를 쓸 위험이 있었음
        # 그래서 이 함수 시작 시 받아올 수 있도록 변경
        self.danger_ob = {}

        # opt 각도 범위 내 값들만 골라냄 (라이다 raw 기준(z축 inverted) -70도는 )
        start_idx = int((math.radians(110)) / (self.angle_increment + 0.00001))
        end_idx = int((math.radians(250)) / (self.angle_increment + 0.00001))
        ranges = self.ranges[start_idx : (end_idx + 1)]
        angle_min = self.angle_min
        angle_increment = self.angle_increment

        # for idx, r in enumerate(ranges):
        #     index = idx + start_idx
        #     pi = -math.degrees(angle_min + angle_increment * index) + 180
        #     print("idx {} pi {} -> {} r {}".format(idx, math.degrees(angle_min + angle_increment * index), rearrange_angle(pi), r))

        # for idx, r in enumerate(self.ranges):
        #     index = idx
        #     pi = -math.degrees(angle_min + angle_increment * index) + 180
        #     print("idx {} pi {} r {}".format(idx, rearrange_angle(pi), r))
        #     # pi = math.degrees(angle_min + angle_increment * index)
        #     # print("idx {} pi {} r {}".format(idx, pi, r))

        if ranges == [] or min(ranges) == float("inf"):
            return False

        closest_distance = min(ranges)  # 가장 가까운 장애물까지 거리
        idx = ranges.index(closest_distance) + start_idx  # 가장 가까운 장애물의 인덱스
        # pi = -math.degrees(angle_min + angle_increment * idx) + 180
        pi = math.degrees(angle_min + angle_increment * idx)
        # lidar는 후방이 0 -> 왼쪽으로 돌아 전방이 180 -> 후방이 360
        pi = rearrange_angle(pi)

        if (0.3 <= closest_distance <= 2.8) and (-70 <= pi <= 70):
            # 장애물이 나로부터 2.8m 이하로 있고, 각도가 좌우 70도 이내일 때
            self.target_servo_ang.input["distance"] = float(closest_distance)
            self.target_servo_ang.input["angle"] = float(pi)
            self.target_servo_ang.compute()
            self.fuzzy_servo_control = int(self.target_servo_ang.output["target_servo"])

            self.danger_ob["distance"] = closest_distance
            self.danger_ob["idx"] = idx
            self.danger_ob["pi"] = pi

            return True
        else:
            return False

    def print_status(self, is_lpp, u_servo):
        if self.print_cnt < 5:
            self.print_cnt += 1
            return
        else:
            self.print_cnt = 0

        # LPP인지 GPP인지
        mode = "Lpp" if is_lpp else "Gpp"
        print("-" * 70)
        print("")
        print("Mode | {}".format(mode))
        print("")

        # goal이 선수의 어느 쪽에 있는지
        psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"

        # 가장 가까운 점의 위치 및 거리
        if len(self.danger_ob) == 0:
            print("Dangerous Point | {:>4} deg / {:>2} m".format("None", "None"))
        else:
            print(
                "Dangerous Point | {:>4.2f} deg / {:>2.1f} m".format(self.danger_ob["pi"], self.danger_ob["distance"])
            )
        print("")

        # 어느 쪽으로 움직일 건지
        if u_servo < self.servo_middle:
            error_angle_dir_str = "( M )"
        elif u_servo < self.servo_middle:
            error_angle_dir_str = "( R )"
        else:
            error_angle_dir_str = "( L )"

        # 서보 얼마나 움직일지
        if u_servo > self.servo_middle:
            servo_value_str = "<" * ((self.servo_middle - u_servo) // 10)  # go left
        else:
            servo_value_str = ">" * ((self.servo_middle - u_servo) // 10)  # go right

        print("| {:^9} | {:^9} | {:^10} |".format("heading", "goal", "delta_servo"))
        print("{:=^38}".format(""))
        print(
            "| {:>9.2f} | {:>9.2f} | {:>4d} {:5} |".format(
                self.psi, self.psi_goal, self.servo_middle - int(u_servo), error_angle_dir_str
            )
        )
        print("| {:9} | {:^9} | {:^10} |".format("", psi_goal_dir_str, servo_value_str))
        print("")

        # 남은 거리
        print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
        print("")
        print("-" * 70)

    def visualize(self):
        ids = list(range(0, 1000))

        # 목표 지점
        goal_txt = visual.text_rviz(
            name="goal",
            id=ids.pop(),
            x=self.goal_x,
            y=self.goal_y,
            text="({:>4.2f}, {:>4.2f})".format(self.goal_x, self.goal_y),
        )
        goal = visual.point_rviz(
            name="goal",
            id=ids.pop(),
            x=self.goal_x,
            y=self.goal_y,
            color_r=165,
            color_g=242,
            color_b=87,
            scale=0.2,
        )

        # goal_range (도착 인정 범위)
        goal_range = visual.cylinder_rviz(
            name="waypoints",
            id=ids.pop(),
            x=self.goal_x,
            y=self.goal_y,
            scale=self.goal_range * 2,
            color_r=165,
            color_g=242,
            color_b=87,
        )

        # 지나온 경로
        traj = visual.points_rviz(name="traj", id=ids.pop(), points=self.trajectory, color_g=180, scale=0.05)

        # heading
        psi_arrow_end_x = 2 * math.cos(math.radians(self.psi)) + self.boat_x
        psi_arrow_end_y = 2 * math.sin(math.radians(self.psi)) + self.boat_y
        psi = visual.arrow_rviz(
            name="psi",
            id=ids.pop(),
            x1=self.boat_x,
            y1=self.boat_y,
            x2=psi_arrow_end_x,
            y2=psi_arrow_end_y,
            color_r=221,
            color_g=119,
            color_b=252,
        )
        psi_txt = visual.text_rviz(name="psi", id=ids.pop(), text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)

        # 배로부터 목표지점까지 이은 선분
        goal_line = visual.linelist_rviz(
            name="goal_line",
            id=ids.pop(),
            lines=[[self.boat_x, self.boat_y], [self.goal_x, self.goal_y]],
            color_r=91,
            color_g=169,
            color_b=252,
            scale=0.05,
        )

        # danger obstacle
        if len(self.ranges) != 0 and len(self.danger_ob) != 0:
            x = self.boat_x + self.danger_ob["distance"] * math.cos(math.radians(self.psi + self.danger_ob["pi"]))
            y = self.boat_y + self.danger_ob["distance"] * math.sin(math.radians(self.psi + self.danger_ob["pi"]))

            obstacle = visual.point_rviz(
                name="obstacle",
                id=ids.pop(),
                x=x,
                y=y,
                color_r=255,
                scale=0.3,
            )

            to_obstacle = visual.linelist_rviz(
                name="obstacle",
                id=ids.pop(),
                lines=[[self.boat_x, self.boat_y], [x, y]],
                color_r=255,
                scale=0.05,
            )
        else:
            obstacle = visual.del_mark(name="obstacle", id=ids.pop())
            to_obstacle = visual.del_mark(name="obstacle", id=ids.pop())

        # 배와 함께 이동할 X, Y축
        axis_x = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[self.boat_x, self.boat_y], [self.boat_x + 3, self.boat_y]],
            color_r=255,
            scale=0.1,
        )
        axis_y = visual.linelist_rviz(
            name="axis",
            id=ids.pop(),
            lines=[[self.boat_x, self.boat_y], [self.boat_x, self.boat_y + 3]],
            color_g=255,
            scale=0.1,
        )
        axis_x_txt = visual.text_rviz(name="axis", id=ids.pop(), text="X", x=self.boat_x + 3.3, y=self.boat_y)
        axis_y_txt = visual.text_rviz(name="axis", id=ids.pop(), text="Y", x=self.boat_x, y=self.boat_y + 3.3)

        # angle_range (탐색 범위)
        min_angle_x = 2.8 * math.cos(math.radians(self.psi - 70)) + self.boat_x
        min_angle_y = 2.8 * math.sin(math.radians(self.psi - 70)) + self.boat_y
        max_angle_x = 2.8 * math.cos(math.radians(self.psi + 70)) + self.boat_x
        max_angle_y = 2.8 * math.sin(math.radians(self.psi + 70)) + self.boat_y
        angle_range = visual.linelist_rviz(
            name="angle_range",
            id=ids.pop(),
            lines=[
                [self.boat_x, self.boat_y],
                [min_angle_x, min_angle_y],
                [self.boat_x, self.boat_y],
                [max_angle_x, max_angle_y],
            ],
            color_r=160,
            color_g=90,
            color_b=227,
            scale=0.05,
        )

        # lidar raw data
        pcd = []
        to_pcd = []
        pcd_in = []
        for idx, r in enumerate(self.ranges):
            if r != float("inf"):
                pi = -math.degrees(self.angle_min + self.angle_increment * idx) + 180
                pi = rearrange_angle(pi)
                x = self.boat_x + r * math.cos(math.radians(self.psi + pi))
                y = self.boat_y + r * math.sin(math.radians(self.psi + pi))

                if -70 <= pi <= 70:
                    pcd_in.append([x, y])
                    to_pcd.append([self.boat_x, self.boat_y])
                    to_pcd.append([x, y])
                else:
                    pcd.append([x, y])
        all_obs = visual.points_rviz(name="pcd", id=ids.pop(), points=pcd, color_g=100, scale=0.08)
        in_obs = visual.points_rviz(name="pcd", id=ids.pop(), points=pcd_in, color_b=100, scale=0.08)
        to_all_obs = visual.linelist_rviz(
            name="pcd",
            id=ids.pop(),
            lines=to_pcd,
            color_b=100,
            scale=0.02,
        )

        all_markers = visual.marker_array_rviz(
            [
                goal_txt,
                goal,
                psi,
                psi_txt,
                traj,
                goal_line,
                obstacle,
                to_obstacle,
                axis_x,
                axis_y,
                axis_x_txt,
                axis_y_txt,
                goal_range,
                angle_range,
                all_obs,
                in_obs,
                to_all_obs,
            ]
        )
        self.visual_rviz_pub.publish(all_markers)


def main():
    rospy.init_node("fuzzy_ctrl", anonymous=False)
    rate = rospy.Rate(10)
    fuzz = Fuzzy()
    fuzz.fuzzy()  # FUZZY 규칙 등록

    while not fuzz.is_all_connected():
        rospy.sleep(0.2)
    print("\n<<<<<<<<<<<<<<<<<<< All Connected !")

    while not rospy.is_shutdown():
        fuzz.calc_psi_goal()
        arrived = fuzz.arrival_check()
        if arrived:
            fuzz.thruster_pub.publish(1500)  # 정지
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return
        else:
            fuzz.trajectory.append([fuzz.boat_x, fuzz.boat_y])

            is_lpp = fuzz.fuzzy_control_avoidance()

            if is_lpp:
                u_servo = fuzz.servo_middle + fuzz.fuzzy_servo_control
            else:
                u_servo = fuzz.servo_pid_controller()

            fuzz.servo_pub.publish(int(u_servo))
            fuzz.thruster_pub.publish(int(fuzz.thruster))

            fuzz.print_status(is_lpp, u_servo)
            fuzz.visualize()

        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()

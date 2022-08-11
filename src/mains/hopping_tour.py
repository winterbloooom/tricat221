#!/usr/bin/env python
# -*- coding:utf-8 -*-

import copy
import math
import os
import sys

import cv2
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import perception.gnss_converter as gc
import utils.filtering as filtering
import utils.hopping_visualize as hopping_visualize


def rearrange_angle(input_angle):
    if input_angle >= 180:  # 왼쪽으로 회전이 더 이득
        output_angle = -180 + abs(input_angle) % 180
    elif input_angle <= -180:
        output_angle = 180 - abs(input_angle) % 180
    else:
        output_angle = input_angle
    return output_angle


class Hopping:
    def __init__(self):
        self.waypoint_idx = 1  # 지금 향하고 있는 waypoint 번호

        # coordinates
        self.remained_waypoints = {}  # 남은 waypoints. key는 waypoint 순서, value는 [x, y] 좌표
        self.gnss_waypoint = rospy.get_param("waypoints")  # GPS 형식의 전체 waypoints
        for idx, waypoint in enumerate(self.gnss_waypoint):
            e, n = gc.enu_convert(waypoint)  # ENU 좌표계로 변환
            self.remained_waypoints[idx + 1] = [n, e]  # 축이 반대이므로 순서 바꿔 할당.
        self.waypoints = copy.deepcopy(self.remained_waypoints)
        self.boat_y, self.boat_x = 0, 0  # gc.enu_convert(rospy.get_param("origin"))  # 배의 좌표
        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]  # 다음 목표 x좌표
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]  # 다음 목표 y좌표
        self.trajectory = []  # 지금껏 이동한 궤적

        # limits, ranges
        self.goal_range = rospy.get_param("goal_range")

        # PID coefficients
        self.kp_distance = rospy.get_param("kp_distance")  # (0 ~ 100)
        self.ki_distance = rospy.get_param("ki_distance")  # (0 ~ 10)
        self.kd_distance = rospy.get_param("kd_distance")  # (0 ~ 100)

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_desire = 0  # 지구고정좌표계 기준 움직여야 할 각도
        self.psi_goal = 0  # 현재 선수각으로부터 goal까지 가기 위해 움직여야 할 각도. 선박 기준(-180 ~ 180)
        self.error_angle = 0  # 다음 목표까지 가기 위한 차이각
        self.heading_queue = []  # 헤딩을 필터링할 이동평균필터 큐

        # other fix values
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.rotate_angle_range = rospy.get_param("rotate_angle_range")
        self.servo_range = rospy.get_param("servo_range")
        self.servo_middle = int((self.servo_range[0] + self.servo_range[1]) / 2)
        self.thruster_max = rospy.get_param("thruster_max")
        self.thruster_min = rospy.get_param("thruster_min")
        self.controller = rospy.get_param("controller")  # PID trackbar
        self.filter_queue_size = rospy.get_param("filter_queue_size")

        # other variables
        self.distance_to_goal = 100000  # 다음 목표까지 남은 거리
        self.cnt = 0  # 상태 출력을 조절할 카운터
        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        # subscribers
        rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        # presetting
        self.calc_distance_to_goal()
        self.calc_error_angle()

        # make controller
        if self.controller:
            cv2.namedWindow("controller")
            cv2.createTrackbar("p dist", "controller", self.kp_distance, 100, lambda x: x)
            cv2.createTrackbar("i dist", "controller", self.ki_distance, 10, lambda x: x)
            cv2.createTrackbar("d dist", "controller", self.kd_distance, 100, lambda x: x)

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수

        Args:
            msg (Float64) : heading. 0 = magnetic north, (+) = 0~180 deg to right, (-) = 0 ~ -180 deg to left

        Notes:
            * IMU의 예민성으로 인하여 heading 값에 noise가 있음. 따라서 이동평균필터를 적용함.
        """
        # self.psi = filtering.moving_avg_filter(
        #     self.heading_queue, self.filter_queue_size, msg.data
        # )  # [deg]
        self.psi = msg.data

    def boat_position_callback(self, msg):
        """GPS로 측정한 배의 ENU 변환 좌표 콜백함수

        Args:
            msg (Point) : position of boat

        Note:
            * ENU좌표계로 변환되어 입력을 받는데, ENU좌표계와 x, y축이 반대임
            * 따라서 Point.x, Point.y가 각각 y, x가 됨
        """
        self.boat_y = msg.x
        self.boat_x = msg.y

    def calc_distance_to_goal(self):
        """calculate distance from boat to goal"""
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)

    def distance_PID(self):
        """calculate thruster control value with PID Contol
        Note
            * thruster는 속도를 결정하므로, 거리에 비례해 속도를 조절함
            * 한 waypoint로 점점 가까이 다가갈수록 속도가 느려짐
            * 단, 다음 point를 찍었을 때는 거리가 멀기 때문에 급출발을 할 수 있음에 주의
            * 여기서는 I 제어 필요 없을 듯해 일단 지워둠
            * m 단위인 distance 쓰러스터 제어값으로 바꾸는 법: 계수값 조정 + min/max 값 더하고 빼고
        """
        self.set_PID_value()
        cp_distance = self.kp_distance * self.distance_to_goal
        cd_distance = -self.kd_distance * self.distance_to_goal / 0.1  # dt = rate

        u_distance = cp_distance + cd_distance
        u_thruster = self.thruster_min + u_distance

        if u_thruster > self.thruster_max:
            u_thruster = self.thruster_max
        if u_thruster < self.thruster_min:
            u_thruster = self.thruster_min

        return int(u_thruster)

    def set_next_goal(self):
        del self.remained_waypoints[self.waypoint_idx]
        self.waypoint_idx += 1

        # print("remain: ", self.remained_waypoints)
        # print("waypont: ", self.gnss_waypoint)
        # print("waypoint_idx", self.waypoint_idx)

        if len(self.gnss_waypoint) + 1 == self.waypoint_idx:
            return

        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]

    def arrival_check(self):
        self.calc_distance_to_goal()  # 목적지까지 거리 다시 계산
        if self.distance_to_goal <= self.goal_range:
            for _ in range(3):  # TODO 잘 작동하는가?
                self.thruster_pub.publish(1500)
                rospy.sleep(1)
            return True
        else:
            return False

    def calc_error_angle(self):
        # hopping에서는 error_angle이 곧 psi_goal임
        self.psi_goal = (
            math.degrees(math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x))
            - self.psi
        )
        self.psi_goal = rearrange_angle(self.psi_goal)
        self.error_angle = self.psi_goal
        self.psi_desire = rearrange_angle(self.psi + self.error_angle)

    def calc_servo_value(self):
        u_angle = (-self.error_angle) * self.angle_alpha
        # 조절 상수 곱해 감도 조절  # 왼쪽이 더 큰 값을 가져야 하므로

        # degree에서 servo로 mapping
        u_servo = (u_angle - self.rotate_angle_range[0]) * (
            self.servo_range[1] - self.servo_range[0]
        ) / (self.rotate_angle_range[1] - self.rotate_angle_range[0]) + self.servo_range[0]

        # servo motor 제어 가능 범위 내부에 머무르도록 함
        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]
        return int(u_servo)

    def set_PID_value(self):
        if self.controller:
            self.kp_distance = cv2.getTrackbarPos("p dist", "controller")
            self.ki_distance = cv2.getTrackbarPos("i dist", "controller")
            self.kd_distance = cv2.getTrackbarPos("d dist", "controller")

    def control_publish(self):
        # 에러각 계산 -> PID로
        self.calc_error_angle()
        self.u_servo = self.calc_servo_value()

        # 남은 거리 계산 -> PID로
        self.calc_distance_to_goal()
        self.u_thruster = self.distance_PID()

        self.servo_pub.publish(int(self.u_servo))
        self.thruster_pub.publish(int(self.u_thruster))

    def print_state(self, visualize=False):
        if self.cnt < 5:
            self.cnt += 1
            return
        else:
            self.cnt = 0

        if self.waypoint_idx > len(self.waypoints):
            return

        print("-" * 40)
        print("Boat [{:>4.2f}, {:>4.2f}]".format(self.boat_x, self.boat_y))
        print(
            "Goal # {} / {}  [{:>4.2f}, {:>4.2f}]".format(
                self.waypoint_idx,
                len(self.waypoints),
                self.remained_waypoints[self.waypoint_idx][0],
                self.remained_waypoints[self.waypoint_idx][1],
            )
        )
        print("{:>9} - {:>9} = {:>7}".format("desire", "psi", "error"))

        if self.error_angle > 0:
            print(
                "({:7.2f}) - ({:7.2f}) = ({:6.2f}) [Right]".format(
                    self.psi_desire, self.psi, self.error_angle
                )
            )
            print("Servo : |-------|--{:->3d}--|".format(self.u_servo))
        else:
            print(
                "({:7.2f}) - ({:7.2f}) = ({:6.2f}) [ Left]".format(
                    self.psi_desire, self.psi, self.error_angle
                )
            )
            print("Servo : |--{:-<3d}--|-------|".format(self.u_servo))

        print("Distance : {:5.2f} m".format(self.distance_to_goal))
        print(
            "Thruster : {:>4d} | P [{:4d}], I [{:>4.1f}], D [{:>4.1f}]".format(
                self.u_thruster, self.kp_distance, self.ki_distance, self.kd_distance
            )
        )
        print("")

        if visualize:
            all_markers = hopping_visualize.visualize(self)
            self.visual_rviz_pub.publish(all_markers)


def main():
    rospy.init_node("HoppingTour", anonymous=False)

    hopping = Hopping()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(hopping.remained_waypoints) == 0:
            # 마지막 목적지까지 도착함
            hopping.servo_pub.publish(hopping.servo_middle)
            hopping.thruster_pub.publish(1500)
            print("-" * 20)
            print("Finished!")
            return
        else:
            hopping.trajectory.append([hopping.boat_x, hopping.boat_y])
            if hopping.arrival_check():
                hopping.set_next_goal()  # 목적지에 도착했음 -> 다음 목적지로 변경
                print("\n##### Arrived Current Goal. Set next goal #####\n")

            hopping.control_publish()  # 계속 다음 목적지로 이동하라

        hopping.print_state(visualize=True)

        if hopping.controller:
            if cv2.waitKey(1) == 27:
                break
        rate.sleep()


if __name__ == "__main__":
    main()

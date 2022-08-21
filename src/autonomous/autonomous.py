#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""Script for autonomous(obstacle avoidance) misson"""

import math
import os
import sys

import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import control.control_tools as control
import datatypes.point_class
import utils.gnss_converter as gc
import utils.obstacle_avoidance as oa
import utils.visualizer as visual
from tricat221.msg import ObstacleList
from utils.tools import *


class Autonomous:
    def __init__(self):
        # locations, coordinates
        self.boat_x, self.boat_y = 0, 0
        self.goal_x, self.goal_y, _ = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.trajectory = []  # 지금까지 이동한 궤적
        boundary = rospy.get_param("boundary2")  # TODO
        self.boundary = []
        for p in boundary:
            self.boundary.append(list(gc.enu_convert(p)))
        self.diff = [0, 0]

        # directions
        self.heading_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 의미 변화함: 나로부터 goal이 떨어진 각도. (+)면 오른쪽, (-)면 왼쪽에 있음
        self.psi_desire = 0  # 이번에 가야 할 각도

        # obstacles
        self.ob_dist_range = rospy.get_param("ob_dist_range")  # 라이다로 장애물을 탐지할 최대거리
        self.ob_angle_range = rospy.get_param("ob_angle_range")  # 장애물 탐지 각도
        self.span_angle = rospy.get_param("span_angle")  # 장애물 양쪽에 더해줄 각도 여유분. 충돌 방지용
        self.input_points = []  # lidar raw data
        self.obstacles = []  # 장애물 전체
        self.inrange_obstacles = []  # 탐지 범위 내 장애물
        self.danger_angles = []  # 장애물이 존재하는 각도 리스트

        # distances
        self.goal_range = rospy.get_param("goal_range")  # 도착이라 판단할 거리(반지름)
        self.distance_to_goal = 100000  # 배~목적지 거리. max 연산이므로 큰 값을 초기 할당

        # control
        self.rotate_angle_range = rospy.get_param("rotate_angle_range")  # 회전할 각도 범위
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.filter_queue = []  # 서보모터값을 필터링할 이동평균필터 큐
        self.thruster_speed = rospy.get_param("thruster_speed")  # 쓰러스터 고정값

        # other fixed values
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.angle_alpha = rospy.get_param("angle_alpha")  # angle_to_servo 함수에서 사용할 상수

        # visualize
        self.show_raw_pcd = rospy.get_param("show_raw_pcd")  # 라이다 raw 데이터 보이기
        self.print_cnt = 0  # 출력 속도를 조정할 카운터.

        # subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        if self.show_raw_pcd:
            self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        # pre-setting
        self.arrival_check()  # 다음 목표까지 남은 거리

    def scan_callback(self, msg):
        if not self.show_raw_pcd:
            return

        self.input_points = []
        phi = msg.angle_min  # 각 점의 각도 계산 위해 계속 누적해갈 각도
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                p = datatypes.point_class.Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수

        Args:
            msg (Float64) : heading. 0 = magnetic north, (+) = 0~180 deg to right, (-) = 0 ~ -180 deg to left

        Notes:
            * IMU의 예민성으로 인하여 heading 값에 noise가 있음. 따라서 이동평균필터를 적용함.
        """

        # self.psi = moving_avg_filter(
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
        self.boat_x = msg.x + self.diff[1]
        self.boat_y = msg.y + self.diff[0]

    def obstacle_callback(self, msg):
        """lidar_converter에서 받아온 장애물 정보 저장

        Args:
            msg (ObstacleList) : list of obstacles(Obstacle object)

        Note:
            * 개당 [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]
        """
        self.obstacles = msg.obstacle

    def is_all_connected(self):
        """make sure all subscribers(nodes) are connected to this node

        Returns:
            bool : True(if all connected) / False(not ALL connected yet)

        Notes:
            * 방법1: `if self.heading_sub.get_num_connections() == 0` - 노드 연결 여부만 체크
            * 방법2: `rospy.wait_for_message("/heading", Float64)` - 값 들어올 때까지 무한정 대기
        """
        ## 방법1
        # not_connected = ""  # 아직 연결되지 않은 센서 목록
        # if self.heading_sub.get_num_connections() == 0:
        #     not_connected += "headingCalculator\t"
        # if self.enu_pos_sub.get_num_connections() == 0:
        #     not_connected += "gnssConverter\t"
        # if self.obstacle_sub.get_num_connections() == 0:
        #     not_connected += "lidarConverter\t"

        # if len(not_connected) == 0:
        #     return True  # 전부 연결됨
        # else:
        #     print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        #     print(not_connected)
        #     print("\n")
        #     return False  # 아직 다 연결되지는 않음

        ## 방법2
        rospy.wait_for_message("/heading", Float64)
        print("\n{:><70}".format("heading_calculator Connected "))
        rospy.wait_for_message("/enu_position", Point)
        print("\n{:><70}".format("gnss_converter Connected "))
        rospy.wait_for_message("/obstacles", ObstacleList)
        print("\n{:><70}".format("lidar_converter Connected "))
        if self.show_raw_pcd:
            rospy.wait_for_message("/scan", LaserScan)
            print("\n{:><70}".format("LiDAR Connected "))
        return True

    def arrival_check(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        return self.distance_to_goal <= self.goal_range

    def print_status(self, error_angle, u_servo):
        """print current state

        Args:
            error_angle (float) : angle between psi_desire and psi (heading to desire angle)
            u_servo (int) : servo moter publish value
        """
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("Obstacle  : {:2d} / {:2d}".format(len(self.inrange_obstacles), len(self.obstacles)))

        psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
        error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
        if u_servo > self.servo_middle:
            servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
        else:
            servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

        print("")
        print("{:^9}   {:^6} - {:^6} = {:^6} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
        print(
            "{:>9}   {:>6.2f} - {:>6.2f} = {:>6.2f} {:>9} {:>5} ( {:^5} )".format(
                psi_goal_dir_str,
                self.psi_desire,
                self.psi,
                error_angle,
                error_angle_dir_str,
                u_servo,
                servo_value_str,
            )
        )
        print("")
        print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
        print("")
        print("-" * 70)

    def visualize(self):
        ids = list(range(0, 100))

        # 경기장
        boundary = visual.linelist_rviz(
            name="boundary",
            id=ids.pop(),
            lines=[
                self.boundary[0],
                self.boundary[1],
                self.boundary[1],
                self.boundary[2],
                self.boundary[2],
                self.boundary[3],
                self.boundary[3],
                self.boundary[0],
            ],
            color_r=59,
            color_g=196,
            color_b=212,
            scale=0.15,
        )

        # 목표 지점
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
        goal_txt = visual.text_rviz(
            name="goal",
            id=ids.pop(),
            x=self.goal_x,
            y=self.goal_y,
            text="({:>4.2f}, {:>4.2f})".format(self.goal_x, self.goal_y),
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

        # 현재 위치
        boat_txt = visual.text_rviz(
            name="boat",
            id=ids.pop(),
            text="({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y),
            x=self.boat_x - 0.3,
            y=self.boat_y - 0.3,
        )

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
        axis_x_txt = visual.text_rviz(name="axis", id=14, text="X", x=self.boat_x + 3.3, y=self.boat_y)
        axis_y_txt = visual.text_rviz(name="axis", id=15, text="Y", x=self.boat_x, y=self.boat_y + 3.3)

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
        psi_txt = visual.text_rviz(name="psi", id=5, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)

        # psi_desire (가고 싶은 각도)
        desire_arrow_end_x = 3 * math.cos(math.radians(self.psi_desire)) + self.boat_x
        desire_arrow_end_y = 3 * math.sin(math.radians(self.psi_desire)) + self.boat_y
        psi_desire = visual.arrow_rviz(
            name="psi_desire",
            id=ids.pop(),
            x1=self.boat_x,
            y1=self.boat_y,
            x2=desire_arrow_end_x,
            y2=desire_arrow_end_y,
            color_r=59,
            color_g=139,
            color_b=245,
        )
        psi_desire_txt = visual.text_rviz(
            name="psi_desire", id=ids.pop(), text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
        )

        # danger_angles
        dangers = []
        for angle in self.danger_angles:
            end_point_x = self.ob_dist_range * math.cos(math.radians(self.psi + angle)) + self.boat_x
            end_point_y = self.ob_dist_range * math.sin(math.radians(self.psi + angle)) + self.boat_y
            dangers.append([self.boat_x, self.boat_y])
            dangers.append([end_point_x, end_point_y])
        danger_angles = visual.linelist_rviz(
            name="obs", id=ids.pop(), lines=dangers, color_r=217, color_g=217, color_b=43, color_a=100, scale=0.02
        )

        # inrange obstacles
        inrange_obs_world = []  # span 미포함
        for ob in self.inrange_obstacles:
            begin_x = (
                self.boat_x
                + (-ob.begin.x) * math.cos(math.radians(self.psi))
                - ob.begin.y * math.sin(math.radians(self.psi))
            )
            begin_y = (
                self.boat_y
                + (-ob.begin.x) * math.sin(math.radians(self.psi))
                + ob.begin.y * math.cos(math.radians(self.psi))
            )
            end_x = (
                self.boat_x
                + (-ob.end.x) * math.cos(math.radians(self.psi))
                - ob.end.y * math.sin(math.radians(self.psi))
            )
            end_y = (
                self.boat_y
                + (-ob.end.x) * math.sin(math.radians(self.psi))
                + ob.end.y * math.cos(math.radians(self.psi))
            )
            inrange_obs_world.append([begin_x, begin_y])
            inrange_obs_world.append([end_x, end_y])
        obstacles = visual.linelist_rviz(
            name="obs", id=ids.pop(), lines=inrange_obs_world, color_r=237, color_g=234, color_b=74, scale=0.1
        )

        # angle_range (탐색 범위)
        min_angle_x = self.ob_dist_range * math.cos(math.radians(self.psi + self.ob_angle_range[0])) + self.boat_x
        min_angle_y = self.ob_dist_range * math.sin(math.radians(self.psi + self.ob_angle_range[0])) + self.boat_y
        max_angle_x = self.ob_dist_range * math.cos(math.radians(self.psi + self.ob_angle_range[1])) + self.boat_x
        max_angle_y = self.ob_dist_range * math.sin(math.radians(self.psi + self.ob_angle_range[1])) + self.boat_y
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
        if self.show_raw_pcd:
            for p in self.input_points:
                x = self.boat_x + (-p.x) * math.cos(math.radians(self.psi)) - p.y * math.sin(math.radians(self.psi))
                y = self.boat_y + (-p.x) * math.sin(math.radians(self.psi)) + p.y * math.cos(math.radians(self.psi))
                pcd.append([x, y])
        pcd = visual.points_rviz(name="pcd", id=ids.pop(), points=pcd, color_r=255, scale=0.08)

        all_markers = visual.marker_array_rviz(
            [
                boundary,
                goal,
                goal_txt,
                goal_range,
                boat_txt,
                traj,
                goal_line,
                axis_x,
                axis_y,
                axis_x_txt,
                axis_y_txt,
                psi,
                psi_txt,
                psi_desire,
                psi_desire_txt,
                danger_angles,
                obstacles,
                angle_range,
                pcd,
            ]
        )
        self.visual_rviz_pub.publish(all_markers)


def main():
    rospy.init_node("autonomous", anonymous=False)
    start_time = rospy.get_time()
    auto = Autonomous()
    rate = rospy.Rate(10)

    while not auto.is_all_connected():
        rospy.sleep(0.2)
    print("\n{:<>70}".format(" All Connected !"))

    while not rospy.is_shutdown():
        arrived = auto.arrival_check()  # 현 시점에서 목표까지 남은 거리 재계산
        if arrived:  # 최종 목적지 도달함
            auto.thruster_pub.publish(1500)  # 정지
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return
        else:
            auto.trajectory.append([auto.boat_x, auto.boat_y])  # 이동 경로 추가

            # 현재 heading에서 목표로 갈 때 돌려야 할 각도 업데이트
            auto.psi_goal = math.degrees(math.atan2(auto.goal_y - auto.boat_y, auto.goal_x - auto.boat_x)) - auto.psi
            auto.psi_goal = rearrange_angle(auto.psi_goal)

            # 장애물 탐지. 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
            auto.inrange_obstacles, auto.danger_angles = oa.ob_filtering(
                obstacles=auto.obstacles,
                dist_to_goal=auto.distance_to_goal,
                angle_to_goal=auto.psi_goal,
                span_angle=auto.span_angle,
                angle_range=auto.ob_angle_range,
                distance_range=auto.ob_dist_range,
            )

            # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            error_angle = oa.calc_desire_angle(
                danger_angles=auto.danger_angles,
                angle_to_goal=auto.psi_goal,
                angle_range=auto.ob_angle_range,
            )

            # 월드좌표계로 '가야 할 각도'를 계산함
            auto.psi_desire = rearrange_angle(auto.psi + error_angle)

            # degree 단위를 servo moter 단위로 변경
            u_servo = control.degree_to_servo(
                error_angle=error_angle,
                angle_alpha=auto.angle_alpha,
                angle_range=auto.rotate_angle_range,
                servo_range=auto.servo_range,
            )
            # u_servo = moving_avg_filter(auto.filter_queue, auto.filter_queue_size, u_servo)

            # 제어명령
            auto.servo_pub.publish(u_servo)
            auto.thruster_pub.publish(auto.thruster_speed)

            # 현 상태 출력 및 시각화
            print("")
            print("{:<9} : {:<6.3f}".format("Run time", rospy.get_time() - start_time))  # 작동 시간
            auto.print_status(error_angle, u_servo)
            auto.visualize()

        rate.sleep()


if __name__ == "__main__":
    main()

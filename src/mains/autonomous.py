#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
Todo:
    * 장애물이 따라가는 것 같음
    * rviz에 텍스트 추가
    * 장애물도 cylinder로 처리. split 단위 알아보고
    * 목표점보다 장애물이 멀리 있을 때(바로 목표점으로 가도 될 때)의 처리가 미완
"""

import math
import os
import sys

import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import obstacle.obstacle_avoidance as oa
import perception.gnss_converter as gc
import utils.filtering as filtering
import utils.visualizer as visual
from tricat221.msg import ObstacleList


class Autonomous:
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

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        # locations, coordinates
        self.boat_x, self.boat_y = 0, 0
        self.goal_y, self.goal_x = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.trajectory = []  # 지금까지 이동한 궤적

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 의미 변화함: 나로부터 goal이 떨어진 각도. (+)면 오른쪽, (-)면 왼쪽에 있음
        self.psi_desire = 0  # 이번에 가야 할 각도
        self.error_angle = 0  # psi와 psi_desire 사이의 각도

        # ranges, limits
        self.goal_range = rospy.get_param("goal_range")  # 도착이라 판단할 거리(반지름)
        self.ob_dist_range = rospy.get_param("ob_dist_range")  # 라이다로 장애물을 탐지할 최대거리
        self.ob_angle_range = rospy.get_param("ob_angle_range")  # 장애물 탐지 각도
        self.rotate_angle_range = rospy.get_param("rotate_angle_range")  # 회전할 각도 범위
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.thruster_speed = rospy.get_param("thruster_speed")  # 쓰러스터 고정값
        self.span_angle = rospy.get_param("span_angle")  # 장애물 양쪽에 더해줄 각도 여유분. 충돌 방지용

        # other fixed values
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.angle_alpha = rospy.get_param("angle_alpha")  # angle_to_servo 함수에서 사용할 상수

        # other variables
        self.filter_queue = []  # 서보모터값을 필터링할 이동평균필터 큐
        self.heading_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.distance_to_goal = 100000  # 배~목적지 거리. max 연산이므로 큰 값을 초기 할당
        self.print_cnt = 0  # 출력 속도를 조정할 카운터.
        self.obstacles = []  # 장애물 전체
        self.inrange_obstacles = []  # 탐지 범위 내 장애물
        self.danger_angles = []  # 장애물이 존재하는 각도 리스트

        # pre-setting
        self.arrival_check()  # 다음 목표까지 남은 거리

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수

        Args:
            msg (Float64) : heading. 0 = magnetic north, (+) = 0~180 deg to right, (-) = 0 ~ -180 deg to left

        Notes:
            * IMU의 예민성으로 인하여 heading 값에 noise가 있음. 따라서 이동평균필터를 적용함.
        """
        self.psi = filtering.moving_avg_filter(
            self.heading_queue, self.filter_queue_size, msg.data
        )  # [deg]

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
        """
        not_connected = ""  # 아직 연결되지 않은 센서 목록
        if self.heading_sub.get_num_connections() == 0:
            not_connected += "headingCalculator\t"
        if self.enu_pos_sub.get_num_connections() == 0:
            not_connected += "gnssConverter\t"
        if self.obstacle_sub.get_num_connections() == 0:
            not_connected += "lidarConverter\t"

        if len(not_connected) == 0:
            return True  # 전부 연결됨
        else:
            print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            print(not_connected)
            print("\n")
            return False  # 아직 다 연결되지는 않음

    def arrival_check(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        return self.distance_to_goal <= self.goal_range

    def show(self, error_angle, u_servo, visualize=False):
        """print current state and visualize them

        Args:
            error_angle (float) : angle between psi_desire and psi (heading to desire angle)
            u_servo (int) : servo moter publish value
            visulize (bool) : visualize with rviz or not
        """
        # show in terminal
        print("-" * 50)
        print("{:>9} - {:>9} = {:>7}".format("desire", "psi", "error"))
        print("({:7.2f}) - ({:7.2f}) = ({:6.2f})".format(self.psi_desire, self.psi, self.error_angle))
        if self.psi_goal - self.psi > 0:
            print(
                "psi_goal : {:7.2f} [Right] | dist : {:6.2f} m".format(
                    self.psi_goal, self.distance_to_goal
                )
            )
        else:
            print(
                "psi_goal : {:7.2f} [ Left] | dist : {:6.2f} m".format(
                    self.psi_goal, self.distance_to_goal
                )
            )
        print("Obstacle : {:2d} / {:2d}".format(len(self.inrange_obstacles), len(self.obstacles)))
        if error_angle > 0:
            print("Move     : Right | Servo: {}".format(u_servo))
        elif error_angle < 0:
            print("Move     : Left  | Servo: {}".format(u_servo))

        # visualize with Rviz
        if visualize:
            # danger_angles
            dangers = []
            for angle in self.danger_angles:
                end_point_x = (
                    self.ob_dist_range * math.cos(math.radians(self.psi + angle)) + self.boat_x
                )
                end_point_y = (
                    self.ob_dist_range * math.sin(math.radians(self.psi + angle)) + self.boat_y
                )
                dangers.append([self.boat_x, self.boat_y])
                dangers.append([end_point_x, end_point_y])
            danger_angles = visual.linelist_rviz(
                name="obs",
                id=0,
                lines=dangers,
                color_r=217,
                color_g=217,
                color_b=43,
                color_a=100,
                scale=0.02,
            )

            # 목표 지점
            goal_txt = visual.text_rviz(
                name="goal",
                id=1,
                x=self.goal_x,
                y=self.goal_y,
                text="({:>4.2f}, {:>4.2f})".format(self.goal_x, self.goal_y),
            )
            goal = visual.point_rviz(
                name="goal",
                id=2,
                x=self.goal_x,
                y=self.goal_y,
                color_r=165,
                color_g=242,
                color_b=87,
                scale=0.2,
            )
            # 지나온 경로
            traj = visual.points_rviz(
                name="traj", id=3, points=self.trajectory, color_g=180, scale=0.05
            )

            # heading
            psi_arrow_end_x = 2 * math.cos(math.radians(self.psi)) + self.boat_x
            psi_arrow_end_y = 2 * math.sin(math.radians(self.psi)) + self.boat_y
            psi = visual.arrow_rviz(
                name="psi",
                id=4,
                x1=self.boat_x,
                y1=self.boat_y,
                x2=psi_arrow_end_x,
                y2=psi_arrow_end_y,
                color_r=221,
                color_g=119,
                color_b=252,
            )
            psi_txt = visual.text_rviz(
                name="psi", id=5, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y
            )

            # psi_desire (가고 싶은 각도)
            desire_arrow_end_x = 3 * math.cos(math.radians(self.psi_desire)) + self.boat_x
            desire_arrow_end_y = 3 * math.sin(math.radians(self.psi_desire)) + self.boat_y
            psi_desire = visual.arrow_rviz(
                name="psi_desire",
                id=6,
                x1=self.boat_x,
                y1=self.boat_y,
                x2=desire_arrow_end_x,
                y2=desire_arrow_end_y,
                color_r=59,
                color_g=139,
                color_b=245,
            )
            psi_desire_txt = visual.text_rviz(
                name="psi_desire", id=7, text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
            )

            # 배로부터 목표지점까지 이은 선분
            goal_line = visual.linelist_rviz(
                name="goal_line",
                id=8,
                lines=[[self.boat_x, self.boat_y], [self.goal_x, self.goal_y]],
                color_r=91,
                color_g=169,
                color_b=252,
                scale=0.05,
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
                name="obs",
                id=9,
                lines=inrange_obs_world,
                color_r=237,
                color_g=234,
                color_b=74,
                scale=0.1,
            )

            # 배와 함께 이동할 X, Y축
            axis_x = visual.linelist_rviz(
                name="axis",
                id=10,
                lines=[[self.boat_x, self.boat_y], [self.boat_x + 3, self.boat_y]],
                color_r=255,
                scale=0.1,
            )
            axis_y = visual.linelist_rviz(
                name="axis",
                id=11,
                lines=[[self.boat_x, self.boat_y], [self.boat_x, self.boat_y + 3]],
                color_g=255,
                scale=0.1,
            )
            axis_x_txt = visual.text_rviz(
                name="axis", id=14, text="X", x=self.boat_x + 3.3, y=self.boat_y
            )
            axis_y_txt = visual.text_rviz(
                name="axis", id=15, text="Y", x=self.boat_x, y=self.boat_y + 3.3
            )

            # goal_range (도착 인정 범위)
            goal_range = visual.cylinder_rviz(
                name="waypoints",
                id=12,
                x=self.goal_x,
                y=self.goal_y,
                scale=self.goal_range * 2,
                color_r=165,
                color_g=242,
                color_b=87,
            )

            # angle_range (탐색 범위)
            min_angle_x = (
                self.ob_dist_range * math.cos(math.radians(self.psi + self.ob_angle_range[0]))
                + self.boat_x
            )
            min_angle_y = (
                self.ob_dist_range * math.sin(math.radians(self.psi + self.ob_angle_range[0]))
                + self.boat_y
            )
            max_angle_x = (
                self.ob_dist_range * math.cos(math.radians(self.psi + self.ob_angle_range[1]))
                + self.boat_x
            )
            max_angle_y = (
                self.ob_dist_range * math.sin(math.radians(self.psi + self.ob_angle_range[1]))
                + self.boat_y
            )
            angle_range = visual.linelist_rviz(
                name="angle_range",
                id=13,
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

            all_markers = visual.marker_array_rviz(
                [
                    danger_angles,
                    goal_txt,
                    goal,
                    psi,
                    psi_txt,
                    traj,
                    psi_desire,
                    psi_desire_txt,
                    goal_line,
                    obstacles,
                    axis_x,
                    axis_y,
                    axis_x_txt,
                    axis_y_txt,
                    goal_range,
                    angle_range,
                ]
            )
            self.visual_rviz_pub.publish(all_markers)

    def degree_to_servo(self, error_angle):
        """
        Args:
            error_angle (float): 왼쪽으로 틀어야 하면 -, 오른쪽으로 틀어야 하면 +, 안 움직여도 되면 0

        Todo:
            * docking과 합쳐서 유틸에서 하기

        Note:
                      (x       - input_min        ) * (output_max     - output_min    ) / (input_max         - input_min        ) + output_min
            u_servo = (u_angle - ob_angle_range[0]) * (servo_range[1] - servo_range[0]) / (ob_angle_range[1] - ob_angle_range[0]) + servo_range[0]
        """
        # angle_mid = sum(self.ob_angle_range) / 2  # 중앙 각도
        # u_angle = angle_mid - error_angle  # 중앙(heading=0으로 두고)으로부터 돌려야 할 각도

        u_angle = -error_angle  # 왼쪽이 더 큰 값을 가져야 하므로

        u_servo = (u_angle - self.rotate_angle_range[0]) * (
            self.servo_range[1] - self.servo_range[0]
        ) / (self.rotate_angle_range[1] - self.rotate_angle_range[0]) + self.servo_range[
            0
        ]  # degree에서 servo로 mapping
        u_servo *= self.angle_alpha  # 조절 상수 곱해 감도 조절

        if u_servo > self.servo_range[1]:
            u_servo = self.servo_range[1]
        elif u_servo < self.servo_range[0]:
            u_servo = self.servo_range[0]
        return int(u_servo)


def main():
    rospy.init_node("autonomous", anonymous=False)
    auto = Autonomous()
    rate = rospy.Rate(10)

    while not auto.is_all_connected():
        rospy.sleep(0.2)
    print("\n<<<<<<<<<<<<<<<<<<< All Connected !")

    while not rospy.is_shutdown():
        arrived = auto.arrival_check()  # 현 시점에서 목표까지 남은 거리 재계산
        if arrived:  # 최종 목적지 도달함
            auto.thruster_pub.publish(1500)  # 정지
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return
        else:
            auto.trajectory.append([auto.boat_x, auto.boat_y])  # 이동 경로 추가
            auto.psi_goal = math.degrees(
                math.atan2(auto.goal_y - auto.boat_y, auto.goal_x - auto.boat_x)
            )  # 목표까지 떨어진 각도 갱신

            auto.inrange_obstacles, auto.danger_angles = oa.ob_filtering(
                auto.obstacles,
                auto.distance_to_goal,
                auto.psi_goal - auto.psi,
                auto.span_angle,
                auto.ob_angle_range,
                auto.ob_dist_range,
            )  # 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
            error_angle = oa.calc_desire_angle(
                auto.danger_angles, auto.psi_goal - auto.psi, auto.ob_angle_range
            )  # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            # TODO error angle이 180, -180 넘어가는 범위 나오는지 잘 체크하기!
            auto.psi_desire = auto.psi + error_angle  # 월드좌표계로 '가야 할 각도'를 계산함

            u_servo = auto.degree_to_servo(error_angle)  # degree 단위를 servo moter 단위로 변경
            u_servo = filtering.moving_avg_filter(
                auto.filter_queue, auto.filter_queue_size, u_servo
            )  # 이동평균필터링

            auto.servo_pub.publish(u_servo)
            auto.thruster_pub.publish(auto.thruster_speed)

            auto.show(error_angle, u_servo, visualize=True)  # 현 상태 출력

        rate.sleep()


if __name__ == "__main__":
    main()

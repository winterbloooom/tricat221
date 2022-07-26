#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
Todo:
    * 장애물이 따라가는 것 같음
    * rviz에 텍스트 추가
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
        # self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        # locations, coordinates
        self.boat_x, self.boat_y = 0, 0
        self.goal_y, self.goal_x = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.trajectory = []

        # directions
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 한 번 넣으면 변하지 않는 값임 # TODO 아닌데??
        self.psi_desire = 0  # 이번에 가야 할 각도
        self.error_angle = 0  # psi와 psi_desire 사이의 각도

        # ranges, limits
        self.goal_range = rospy.get_param("goal_range")
        self.ob_dist_range = rospy.get_param("ob_dist_range")
        self.ob_angle_range = rospy.get_param("ob_angle_range")  # 장애물 탐지 각도 = 이동 가능 각도
        self.servo_range = rospy.get_param("servo_range")
        self.thruster_speed = rospy.get_param("thruster_speed")
        self.span_angle = rospy.get_param("span_angle")

        # values
        self.distance_to_goal = 100000

        self.print_cnt = 0
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.filter_queue = [0] * self.filter_queue_size
        self.angle_alpha = rospy.get_param("angle_alpha")

        # obstacles
        ## 내 패키지 사용, 구분 없음
        self.obstacles = []
        self.inrange_obstacles = []
        self.danger_angles = []

        # pre-setting
        self.arrival_check()  # 다음 목표까지 남은 거리

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수"""
        self.psi = msg.data  # [degree]

    # def yaw_rate_callback(self, msg):
    #     self.yaw_rate = math.degrees(msg.angular_velocity.z)  # [rad/s] -> [degree/s]

    def boat_position_callback(self, msg):
        """GPS로 측정한 배의 ENU 변환 좌표 콜백함수"""
        self.boat_y = msg.x  # ENU 좌표계와 축이 반대라 바꿔줌
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle
        # [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

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

        if len(not_connected) == 0:
            return True
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")  # TODO 예쁘게
            print(not_connected)  # TODO 예쁘게
            print("\n")
            return False

    def arrival_check(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)

        return self.distance_to_goal <= self.goal_range

    def calc_psi_goal(self):
        self.psi_goal = math.degrees(
            math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)
        )

    def show(self, error_angle, u_servo, visualize=False):
        print("-" * 20)
        print("goal [{} ,{}]".format(self.goal_x, self.goal_y))
        print("psi {} | psi_goal {:.2f}".format(self.psi, self.psi_goal))
        print("psi_desire {} | error {}".format(self.psi_desire, error_angle))
        print("danger obs : {} / {}".format(len(self.inrange_obstacles), len(self.obstacles)))
        if error_angle > 0:
            print("Turn Right | Servo: {}".format(u_servo))
        elif error_angle < 0:
            print("Turn Left | Servo: {}".format(u_servo))

        if visualize:
            # 목표 지점
            goal = visual.point_rviz(
                name="goal", id=2, x=self.goal_x, y=self.goal_y, color_b=255, scale=0.2
            )
            # 지나온 경로
            traj = visual.points_rviz(name="traj", id=3, points=self.trajectory, color_g=255)
            # psi
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
                name="psi", id=8, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y
            )
            # psi_desire
            desire_arrow_end_x = 2 * math.cos(math.radians(self.psi_desire)) + self.boat_x
            desire_arrow_end_y = 2 * math.sin(math.radians(self.psi_desire)) + self.boat_y
            psi_desire = visual.arrow_rviz(
                name="psi_desire",
                id=5,
                x1=self.boat_x,
                y1=self.boat_y,
                x2=desire_arrow_end_x,
                y2=desire_arrow_end_y,
                color_r=59,
                color_g=139,
                color_b=245,
            )
            psi_desire_txt = visual.text_rviz(
                name="psi_desire", id=9, text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
            )
            # 골까지
            goal_line = visual.linelist_rviz(
                name="goal_line",
                id=6,
                lines=[[self.boat_x, self.boat_y], [self.goal_x, self.goal_y]],
                color_b=255,
                scale=0.04,
            )
            # angle_range
            ## 지금은 스킵
            # inrange obs
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
                name="obs", id=7, lines=inrange_obs_world, color_r=237, color_g=234, color_b=74
            )
            # axis
            axis_x = visual.linelist_rviz(
                name="axis_x",
                id=11,
                lines=[[self.boat_x, self.boat_y], [self.boat_x + 3, self.boat_y]],
                color_r=255,
                scale=0.1,
            )
            axis_y = visual.linelist_rviz(
                name="axis_x",
                id=12,
                lines=[[self.boat_x, self.boat_y], [self.boat_x, self.boat_y + 3]],
                color_g=255,
                scale=0.1,
            )
            all_markers = visual.marker_array_rviz(
                [
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
                ]
            )
            self.visual_rviz_pub.publish(all_markers)

    def degree_to_servo(self, error_angle):
        """
        Args:
            error_angle (float): 왼쪽으로 틀어야 하면 -, 오른쪽으로 틀어야 하면 +, 안 움직여도 되면 0
            ob_angle_range (list): [angle_min, angle_max]
            servo_range (list): [servo_min, servo_max]
            alpha (int): 조정 상수

        Todo:
            * 유틸에서 하기

        Note:
                    (x       - input_min     ) * (output_max     - output_min    ) / (input_max      - input_min     ) + output_min
            u_servo = (u_angle - ob_angle_range[0]) * (servo_range[1] - servo_range[0]) / (ob_angle_range[1] - ob_angle_range[0]) + servo_range[0]
        """
        angle_mid = sum(self.ob_angle_range) / 2
        u_angle = angle_mid - error_angle  # 부호 반대여야 하는데, 서보는 왼쪽이 더 커져야 하니까 이렇게 함 -> 여전히 반대인데?
        u_servo = (u_angle - self.ob_angle_range[0]) * (
            self.servo_range[1] - self.servo_range[0]
        ) / (self.ob_angle_range[1] - self.ob_angle_range[0]) + self.servo_range[0]
        return u_servo * self.angle_alpha


def main():
    rospy.init_node("autonomous", anonymous=False)
    auto = Autonomous()
    rate = rospy.Rate(10)

    while not auto.is_all_connected():
        rospy.sleep(0.2)
    print("\n----------All Connected----------\n")

    while not rospy.is_shutdown():
        arrived = auto.arrival_check()  # 현 시점에서 목표까지 남은 거리 재계산
        if arrived:  # 최종 목적지 도달함
            auto.thruster_pub.publish(1500)
            print("Finished")  # TODO 예쁘게
            return
        else:
            auto.trajectory.append([auto.boat_x, auto.boat_y])
            auto.inrange_obstacles, auto.danger_angles = oa.ob_filtering(
                auto.obstacles, auto.span_angle, auto.ob_angle_range, auto.ob_dist_range
            )
            auto.calc_psi_goal()
            error_angle = oa.calc_desire_angle(
                auto.danger_angles, auto.psi_goal - auto.psi, auto.ob_angle_range
            )  # 목표각과 현 헤딩 사이 상대적 각도
            auto.psi_desire = auto.psi + error_angle
            u_servo = auto.degree_to_servo(error_angle)
            u_servo = filtering.moving_avg_filter(auto.filter_queue, u_servo)
            auto.servo_pub.publish(u_servo)
            auto.thruster_pub.publish(auto.thruster_speed)
            auto.show(error_angle, u_servo, visualize=True)
        rate.sleep()


if __name__ == "__main__":
    main()

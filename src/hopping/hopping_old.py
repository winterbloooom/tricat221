#!/usr/bin/env python
# -*- coding:utf-8 -*-

import copy
import math
import os
import sys

import numpy as np
import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import utils.gnss_converter as gc
import utils.visualizer as visual


def RAD2DEG(x):
    return x * 180.0 / math.pi


def DEG2RAD(x):
    return x / 180.0 * math.pi


class Goal:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.trajectory = []  # 지금껏 이동한 궤적
        self.gps_diff = [0, 0]
        boundary = rospy.get_param("boundary1")
        self.boundary = []
        for p in boundary:
            n, e, _ = gc.enu_convert(p)
            self.boundary.append([e, n])

        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/heading", Float64, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)

        ## ENU & Waypoint List
        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = (
            self.map_list["map_00_lat"],
            self.map_list["map_00_lon"],
            self.map_list["map_00_alt"],
        )

        self.waypoint_idx = 1  # 지금 향하고 있는 waypoint 번호
        self.remained_waypoints = {}
        self.gnss_waypoint = rospy.get_param("waypoint_List/waypoints")  # geodetic
        for idx, waypoint in enumerate(self.gnss_waypoint):
            n, e, _ = gc.enu_convert(waypoint)  # ENU 좌표계로 변환
            self.remained_waypoints[idx + 1] = [e, n]
        self.waypoints = copy.deepcopy(self.remained_waypoints)
        print(self.waypoints)

        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]  # 다음 목표 x좌표
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]  # 다음 목표 y좌표

        self.goal_range = rospy.get_param("goal_range")

        ## Direction Search
        self.angle = 0.0
        self.bearing = 0.0

        ## PID
        self.init_servo = 94
        self.servo_control = 0
        self.errSum = 0.0
        self.yaw_rate = 0.0

        self.thrust = 0

        self.kp_servo = rospy.get_param("kp_servo")
        self.ki_servo = rospy.get_param("ki_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster_power = rospy.get_param("thruster_power")
        self.min_power = rospy.get_param("min_power")
        self.kp_distance = rospy.get_param("kp_distance")

        self.Servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

    def yaw_rate_callback(self, data):
        self.yaw_rate = data.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self, data):
        self.bearing = data.data

    def enu_callback(self, data):
        self.boat_y = data.x + self.gps_diff[1]  # East
        self.boat_x = data.y + self.gps_diff[0]  # North

    def OPTIMAL_DIRECTION(self, b):
        dx = self.goal_x - self.boat_x
        dy = self.goal_y - self.boat_y
        self.angle = RAD2DEG(math.atan2(dx, dy))

        if dx >= 0 and dy >= 0:  # Quadrant 1
            if b >= 0:  # right bearing
                self.t = self.angle - b
            elif b < 0:  # left bearing
                if abs(b) < (180 - self.angle):
                    self.t = self.angle - b
                elif abs(b) >= (180 - self.angle):
                    self.t = -(360 - self.angle + b)

        elif dx < 0 and dy >= 0:  # Quadrant 2
            if b >= 0:
                if b < 180 + self.angle:
                    self.t = self.angle - b
                elif b >= 180 + self.angle:
                    self.t = 360 + self.angle - b
            elif b < 0:
                self.t = self.angle - b

        elif dx < 0 and dy < 0:  # Quadrant 3
            if b >= 0:
                if b < 180 + self.angle:
                    self.t = self.angle - b
                elif b >= 180 + self.angle:
                    self.t = 360 + (self.angle - b)
            elif b < 0:
                self.t = self.angle - b

        elif dx >= 0 and dy < 0:  # Quadrant 4
            if b >= 0:
                self.t = self.angle - b
            elif b < 0:
                if abs(b) < 180 - self.angle:
                    self.t = self.angle - b
                elif abs(b) >= 180 - self.angle:
                    self.t = self.angle - b - 360

        return self.t

    def target(self):
        return self.OPTIMAL_DIRECTION(self.bearing)

    def set_next_point(self):
        del self.remained_waypoints[self.waypoint_idx]
        self.waypoint_idx += 1
        if len(self.gnss_waypoint) + 1 == self.waypoint_idx:
            return

        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)

        if self.dist_to_goal <= self.goal_range:
            for _ in range(8):
                self.thruster_pub.publish(1600)
                rospy.sleep(0.1)
            # while abs(self.OPTIMAL_DIRECTION(self.bearing)) > 10:
            #     self.rotate_heading(self.OPTIMAL_DIRECTION(self.bearing))
            #     print(self.OPTIMAL_DIRECTION(self.bearing))
            return True
        else:
            return False

    def prt(self):
        if self.servo_control > 94 + 3:  # left turn
            turn = "left"
        elif self.servo_control < 94 - 3:  # right turn
            turn = "right"
        else:
            turn = "mid"
        print("distance, thruster : ", self.dist_to_goal, self.thrust)
        print("my xy : ", self.boat_x, self.boat_y)
        print("way xy : ", self.goal_x, self.goal_y)
        print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
        print("servo : " + turn, round(self.servo_control))
        print("errSum:", self.errSum)
        print("-------------------------------------")

    def servo_pid_controller(self):
        # P ctrl
        error_angle = self.target()  # deg

        # I ctrl
        self.errSum += error_angle * 0.1

        if self.errSum > 90:
            self.errSum = 90
        elif self.errSum < -90:
            self.errSum = -90
        else:
            pass

        # D ctrl
        yaw_rate = RAD2DEG(self.yaw_rate)  # deg/s

        cp_servo = self.kp_servo * error_angle
        ci_servo = self.ki_servo * self.errSum
        cd_servo = self.kd_servo * -yaw_rate

        servo_pid = -(cp_servo + ci_servo + cd_servo)
        self.servo_control = self.init_servo + servo_pid

        if self.servo_control > 94 + 26:  # 94+24
            self.servo_control = 94 + 26
        elif self.servo_control < 94 - 26:
            self.servo_control = 94 - 26
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        ## propotional ctrl for thruster
        self.thrust = int(self.min_power + self.kp_distance * self.dist_to_goal)
        if self.thrust > self.thruster_power:
            self.thrust = self.thruster_power
        else:
            pass
        self.thruster_pub.publish(self.thrust)
        self.Servo_pub.publish(round(self.servo_pid_controller()))

    def show(self):
        ids = list(range(0, 100))

        boat = visual.text_rviz(
            name="boat",
            id=ids.pop(),
            text="({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y),
            x=self.boat_x - 0.3,
            y=self.boat_y - 0.3,
        )

        traj = visual.points_rviz(name="traj", id=ids.pop(), points=self.trajectory, color_g=255)

        psi_arrow_end_x = 2 * math.sin(math.radians(self.bearing)) + self.boat_x
        psi_arrow_end_y = 2 * math.cos(math.radians(self.bearing)) + self.boat_y
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

        # desire_arrow_end_x = 3 * math.sin(math.radians(self.t)) + self.boat_x
        # desire_arrow_end_y = 3 * math.cos(math.radians(self.t)) + self.boat_y
        # psi_desire = visual.arrow_rviz(
        #     name="psi_desire",
        #     id=ids.pop(),
        #     x1=self.boat_x,
        #     y1=self.boat_y,
        #     x2=desire_arrow_end_x,
        #     y2=desire_arrow_end_y,
        #     color_r=59,
        #     color_g=139,
        #     color_b=245,
        # )
        # psi_desire_txt = visual.text_rviz(
        #     name="psi_desire", id=ids.pop(), text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
        # )

        goal_line = visual.linelist_rviz(
            name="goal_line",
            id=ids.pop(),
            lines=[[self.boat_x, self.boat_y], [self.goal_x, self.goal_y]],
            color_r=91,
            color_g=169,
            color_b=252,
            scale=0.05,
        )

        axis_x = visual.linelist_rviz(
            name="axis_x",
            id=100,
            lines=[[self.boat_x, self.boat_y], [self.boat_x + 3, self.boat_y]],
            color_r=255,
            scale=0.1,
        )
        axis_y = visual.linelist_rviz(
            name="axis_x",
            id=ids.pop(),
            lines=[[self.boat_x, self.boat_y], [self.boat_x, self.boat_y + 3]],
            color_g=255,
            scale=0.1,
        )
        axis_x_txt = visual.text_rviz(name="axis", id=ids.pop(), text="X", x=self.boat_x + 3.3, y=self.boat_y)
        axis_y_txt = visual.text_rviz(name="axis", id=ids.pop(), text="Y", x=self.boat_x, y=self.boat_y + 3.3)

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

        all_markers = visual.marker_array_rviz(
            [
                boat,
                traj,
                psi,
                psi_txt,
                boundary,
                goal_line,
                axis_x,
                axis_y,
                axis_x_txt,
                axis_y_txt,
            ]
        )

        for idx in range(self.waypoint_idx, len(self.waypoints) + 1):
            waypoint = visual.point_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.waypoints[idx][0],
                y=self.waypoints[idx][1],
                color_r=78,
                color_g=166,
                color_b=58,
                scale=0.3,
            )
            visual.marker_array_append_rviz(all_markers, waypoint)

            goal_range = visual.cylinder_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.remained_waypoints[idx][0],
                y=self.remained_waypoints[idx][1],
                scale=self.goal_range * 2,
                color_r=78,
                color_g=166,
                color_b=58,
            )
            visual.marker_array_append_rviz(all_markers, goal_range)

            waypoint_txt = visual.text_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.remained_waypoints[idx][0],
                y=self.remained_waypoints[idx][1],
                text=str(idx),
                scale=1.2,
            )
            visual.marker_array_append_rviz(all_markers, waypoint_txt)

        # passed points
        for idx in range(1, self.waypoint_idx):
            waypoint = visual.point_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.waypoints[idx][0],
                y=self.waypoints[idx][1],
                color_r=66,
                color_g=135,
                color_b=245,
                scale=0.3,
            )
            visual.marker_array_append_rviz(all_markers, waypoint)

            goal_range = visual.cylinder_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.waypoints[idx][0],
                y=self.waypoints[idx][1],
                scale=self.goal_range * 2,
                color_r=66,
                color_g=135,
                color_b=245,
            )
            visual.marker_array_append_rviz(all_markers, goal_range)

            waypoint_txt = visual.text_rviz(
                name="waypoints",
                id=ids.pop(),
                x=self.waypoints[idx][0],
                y=self.waypoints[idx][1],
                text=str(idx),
                scale=1.2,
            )
            visual.marker_array_append_rviz(all_markers, waypoint_txt)

        self.visual_rviz_pub.publish(all_markers)

    def rotate_heading(self, error_angle):
        print("=" * 500)
        u_angle = (-error_angle) * 1.2  # 조절 상수 곱해 감도 조절  # 왼쪽이 더 큰 값을 가져야 하므로

        # degree에서 servo로 mapping
        u_servo = (u_angle - (-80)) * (120 - 70) / (80 - (-80)) + 70

        # 중앙값 근처는 전부 중앙값으로 publish
        servo_middle = 95
        if servo_middle - 2 <= u_servo <= servo_middle + 2:
            u_servo = servo_middle

        # servo motor 제어 가능 범위 내부에 머무르도록 함
        if u_servo > 120:
            u_servo = 120
        elif u_servo < 70:
            u_servo = 70

        self.thruster_pub.publish(1550)
        self.Servo_pub.publish(int(u_servo))


def main():
    rospy.init_node("Hopping_PID_controller", anonymous=False)

    goal = Goal()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(goal.remained_waypoints) == 0:
            # 마지막 목적지까지 도착함
            goal.Servo_pub.publish(95)
            goal.thruster_pub.publish(1500)
            print("-" * 20)
            print("Finished!")
            return
        else:
            goal.trajectory.append([goal.boat_x, goal.boat_y])
            if goal.arrival_check():
                goal.set_next_point()

            goal.errSum = 0.0  # error initialization
            goal.control_publisher()
            goal.prt()
            goal.show()

        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()

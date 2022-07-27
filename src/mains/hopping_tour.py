#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

import cv2
import numpy as np
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import perception.gnss_converter as gc
import utils.visualizer as visual

class Hopping:
    def __init__(self):
        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)
        rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

        ## 변수 초기화
        ### waypoint 좌표(x, y) 리스트
        self.remained_waypoint = {}
        self.gnss_waypoint = rospy.get_param("waypoints")
        # ENU 좌표로 변환
        for idx, waypoint in enumerate(self.gnss_waypoint):
            e, n = gc.enu_convert(waypoint)
            enu_waypoint = [n, e]
            self.remained_waypoint[idx+1] = enu_waypoint

        self.goal_range = rospy.get_param("goal_range")

        self.error_sum_angle = 0
        self.kp_angle = rospy.get_param("kp_angle")  # (0.0 ~ 1.0)
        self.ki_angle = rospy.get_param("ki_angle")  # (0.0 ~ 0.1)
        self.kd_angle = rospy.get_param("kd_angle")  # (0.0 ~ 1.0)

        self.kp_distance = rospy.get_param("kp_distance")  # (0 ~ 100)
        self.ki_distance = rospy.get_param("ki_distance")  # (0 ~ 10)
        self.kd_distance = rospy.get_param("kd_distance")  # (0 ~ 100)

        self.servo_middle = rospy.get_param("servo_middle")
        self.servo_left_max = rospy.get_param("servo_left_max")
        self.servo_right_max = rospy.get_param("servo_right_max")

        self.thruster_max = rospy.get_param("thruster_max")
        self.thruster_min = rospy.get_param("thruster_min")

        ### 측정값
        self.yaw_rate = 0  # z축 각속도 [degree/s]
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_desire = 0

        self.boat_x = 0  # 배의 x좌표
        self.boat_y = 0  # 배의 y좌표

        self.waypoint_idx = 1
        self.goal_x = self.remained_waypoint[self.waypoint_idx][0] #self.remained_waypoint[0][0]  # 다음 목표의 x좌표
        self.goal_y = self.remained_waypoint[self.waypoint_idx][1] #self.remained_waypoint[0][1]  # 다음 목표의 y좌표

        self.trajectory = []

        self.distance_to_goal = 100000
        self.calc_distance_to_goal()  # 다음 목표까지 남은 거리
        self.error_angle = 0
        self.calc_error_angle()

        self.cnt = 0
        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

        self.controller = rospy.get_param("controller")

        if self.controller:
            cv2.namedWindow("controller")
            cv2.createTrackbar("p angle", "controller", 3, 10, self.trackbar_callback)
            cv2.createTrackbar("i angle", "controller", 0, 10, self.trackbar_callback)
            cv2.createTrackbar("d angle", "controller", 0, 10, self.trackbar_callback)
            cv2.createTrackbar("p dist", "controller", 40, 100, self.trackbar_callback)
            cv2.createTrackbar("i dist", "controller", 0, 10, self.trackbar_callback)
            cv2.createTrackbar("d dist", "controller", 0, 100, self.trackbar_callback)


    def trackbar_callback(self, usrdata):
        pass


    # IMU z축 각속도 콜백함수
    def yaw_rate_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z)  # [rad/s] -> [degree/s]


    # IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수
    def heading_callback(self, msg):
        self.psi = msg.data  # [degree]


    # GPS로 측정한 배의 ENU 변환 좌표 콜백함수
    def boat_position_callback(self, msg):
        self.boat_y = msg.x  # ENU 좌표계와 축이 반대라 바꿔줌
        self.boat_x = msg.y


    def calc_distance_to_goal(self):
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)


    def distance_PID(self):
        cp_distance = self.kp_distance * self.distance_to_goal
        # ci_distance = self.ki_distance *  # dt = rate / TODO : 여기는 I 제어 필요 없을 듯?
        cd_distance = -self.kd_distance * self.distance_to_goal / 0.1  # dt = rate

        u_distance = cp_distance + cd_distance  # + ci_distance
        u_thruster = self.thruster_min + u_distance
        # m 단위인 distance 쓰러스터 제어값으로 바꾸는 법: 계수값 조정 + min/max 값 더하고 빼고

        if u_thruster > self.thruster_max:
            u_thruster = self.thruster_max

        if u_thruster < self.thruster_min:
            u_thruster = self.thruster_min

        return int(u_thruster)


    def set_next_goal(self):
        self.waypoint_idx += 1
        if len(self.gnss_waypoint) == self.waypoint_idx:
            return
        
        self.goal_x = self.remained_waypoint[self.waypoint_idx][0]
        self.goal_y = self.remained_waypoint[self.waypoint_idx][1]


    def arrival_check(self):
        self.calc_distance_to_goal()  # 목적지까지 거리 다시 계산
        if self.distance_to_goal <= self.goal_range:
            return True
        else:
            return False


    def calc_error_angle(self):
        # psi_desire 계산(x축(North)과 goal 사이 각)
        self.psi_desire = math.degrees(
            math.atan2(self.goal_y - self.boat_y, self.goal_x - self.boat_x)
        )
        self.error_angle = self.psi_desire - self.psi


    def error_angle_PID(self):
        self.set_PID_value()

        cp_angle = self.kp_angle * self.error_angle

        self.error_sum_angle += self.error_angle * 0.1  # dt = rate
        ci_angle = self.ki_angle * self.error_sum_angle
        # TODO : errorsum 초기화할 위치 선정하기

        cd_angle = -self.kd_angle * self.yaw_rate

        u_angle = cp_angle + ci_angle + cd_angle
        u_servo = self.servo_middle - u_angle
        if u_servo > self.servo_left_max:
            u_servo = self.servo_left_max
        elif u_servo < self.servo_right_max:
            u_servo = self.servo_right_max

        # print("cp_angle : {} / ci_angle : {} / cd_angle : {}".format(cp_angle, ci_angle, cd_angle))
        # print("u_angle : {} / u_servo : {}".format(u_angle, u_servo))

        return int(u_servo)


    def set_PID_value(self):
        if self.controller:
            self.kp_angle = cv2.getTrackbarPos("p angle", "controller") * 0.1
            self.ki_angle = cv2.getTrackbarPos("i angle", "controller") * 0.1
            self.kd_angle = cv2.getTrackbarPos("d angle", "controller") * 0.1
            self.kp_distance = cv2.getTrackbarPos("p dist", "controller")
            self.ki_distance = cv2.getTrackbarPos("i dist", "controller")
            self.kd_distance = cv2.getTrackbarPos("d dist", "controller")


    def control_publish(self):
        # 에러각 계산 -> PID로
        self.calc_error_angle()
        self.u_servo = self.error_angle_PID()

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

        print("-" * 40)
        print("Boat [{:>4.2f}, {:>4.2f}]".format(self.boat_x, self.boat_y))
        print("Goal #{} / {}  [{:>4.2f}, {:>4.2f}]".format(self.waypoint_idx,len(self.gnss_waypoint), self.remained_waypoint[self.waypoint_idx][0], self.remained_waypoint[self.waypoint_idx][1]))
        print("{:>9} - {:>9} = {:>7}".format("psi", "desire", "error"))
        
        if self.error_angle > 0:
            print("({:7.2f}) - ({:7.2f}) = ({:6.2f}) [Right]".format(self.psi, self.psi_desire, self.error_angle))
        else:
            print("({:7.2f}) - ({:7.2f}) = ({:6.2f}) [ Left]".format(self.psi, self.psi_desire, self.error_angle))
        print("Servo    : {:>4d} | P {}, I {}, D {}".format(self.u_servo, self.kp_angle, self.ki_angle, self.kd_angle))

        print("Distance : {:5.2f} m".format(self.distance_to_goal))
        print("Thruster : {:>4d} | P {}, I {}, D {}".format(self.u_thruster, self.kp_distance, self.ki_distance, self.kd_distance))

        if visualize:
            traj = visual.points_rviz(name="traj", id=1, points=self.trajectory, color_g=255)
            psi_arrow_end_x = 2 * math.cos(math.radians(self.psi)) + self.boat_x
            psi_arrow_end_y = 2 * math.sin(math.radians(self.psi)) + self.boat_y
            psi = visual.arrow_rviz(
                name="psi",
                id=2,
                x1=self.boat_x,
                y1=self.boat_y,
                x2=psi_arrow_end_x,
                y2=psi_arrow_end_y,
                color_r=221,
                color_g=119,
                color_b=252,
            )
            psi_txt = visual.text_rviz(
                name="psi", id=3, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y
            )
            desire_arrow_end_x = 2 * math.cos(math.radians(self.psi_desire)) + self.boat_x
            desire_arrow_end_y = 2 * math.sin(math.radians(self.psi_desire)) + self.boat_y
            psi_desire = visual.arrow_rviz(
                name="psi_desire",
                id=4,
                x1=self.boat_x,
                y1=self.boat_y,
                x2=desire_arrow_end_x,
                y2=desire_arrow_end_y,
                color_r=59,
                color_g=139,
                color_b=245,
            )
            psi_desire_txt = visual.text_rviz(
                name="psi_desire", id=5, text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
            )
            goal_line = visual.linelist_rviz(
                name="goal_line",
                id=6,
                lines=[[self.boat_x, self.boat_y], [self.goal_x, self.goal_y]],
                color_r=91, 
                color_g=169,
                color_b=252,
                scale=0.05,
            )
            axis_x = visual.linelist_rviz(
                name="axis_x",
                id=7,
                lines=[[self.boat_x, self.boat_y], [self.boat_x + 3, self.boat_y]],
                color_r=255,
                scale=0.1,
            )
            axis_y = visual.linelist_rviz(
                name="axis_x",
                id=8,
                lines=[[self.boat_x, self.boat_y], [self.boat_x, self.boat_y + 3]],
                color_g=255,
                scale=0.1,
            )
            all_markers = visual.marker_array_rviz(
                [
                    psi,
                    psi_txt,
                    traj,
                    psi_desire,
                    psi_desire_txt,
                    goal_line,
                    axis_x,
                    axis_y,
                ]
            )
            id = 9
            for idx in self.remained_waypoint:
                # TODO 지난 목표점은 흐리게 표시!
                # wpt = key
                # remained_waypoint[wpt] = value
                waypoint = visual.point_rviz(name="waypoints", id=id, x=self.remained_waypoint[idx][0], y=self.remained_waypoint[idx][1], color_r=165, color_g=242, color_b=87, scale=0.3)
                goal_range = visual.cylinder_rviz(name="waypoints", id=id+1, x=self.remained_waypoint[idx][0], y=self.remained_waypoint[idx][1], scale=self.goal_range*2, color_r=165, color_g=242, color_b=87)
                waypoint_txt = visual.text_rviz(name="waypoints", id=id+2, x=self.remained_waypoint[idx][0], y=self.remained_waypoint[idx][1], text=str(idx), scale=1.5)
                visual.marker_array_append_rviz(all_markers, waypoint)
                visual.marker_array_append_rviz(all_markers, goal_range)
                visual.marker_array_append_rviz(all_markers, waypoint_txt)
                id += 3
            self.visual_rviz_pub.publish(all_markers)


def main():
    rospy.init_node("HoppingTour", anonymous=False)

    hopping = Hopping()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(hopping.remained_waypoint) == 0:
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

    rospy.spin()


if __name__ == "__main__":
    main()

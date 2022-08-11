#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

import numpy as np
import pymap3d as pm
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import dock.dock_control as dock_control
import dock.dock_visualize as dock_visualize
import dock.mark_detect as mark_detect
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
        self.obstacle_sub = rospy.Subscriber(
            "/obstacles", ObstacleList, self.obstacle_callback, queue_size=1
        )
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cam_callback)
        self.bridge = CvBridge()

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

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
        self.psi_goal = 0
        self.psi_desire = 0
        self.raw_img = np.zeros((480, 640, 3), dtype=np.uint8)  # row, col, channel
        self.hsv_img = np.zeros((480, 640), dtype=np.uint8)
        self.shape_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.obstacles = []
        self.mark_area = 0

        # target info
        self.target_shape = rospy.get_param("target_shape")
        self.target_color = rospy.get_param("target_color")
        target_color_range = rospy.get_param("color_range")[self.target_color]

        # ranges, limits
        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.arrival_range = rospy.get_param("arrival_range")  # 도착여부 판단할 범위
        self.color_range = np.array(
            [
                [
                    target_color_range["color1_lower"],
                    target_color_range["color2_lower"],
                    target_color_range["color3_lower"],
                ],
                [
                    target_color_range["color1_upper"],
                    target_color_range["color2_upper"],
                    target_color_range["color3_upper"],
                ],
            ]
        )
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.mark_detect_area = rospy.get_param("mark_detect_area")  # 도형이 검출될 최소 넓이
        self.target_detect_area = rospy.get_param("target_detect_area")  # 타겟이라고 인정할 최소 넓이
        self.station_dir = rospy.get_param("station_dir")  # [-180, 180]

        # count
        ## (state 4에서) '이 시간동안(횟수)' 정지(약한 후진)하고 그 뒤에 헤딩 돌릴 것.
        self.stop_time = rospy.get_param("stop_time")
        ## (state 4에서) 몇 번 정지 신호를 보냈는가?
        self.stop_cnt = 0
        ## (state 5에서) '이만큼(횟수)' 기다리며 얼마나 발견하나 횟수를 셈
        self.target_detect_time = rospy.get_param("target_detect_time")
        ## (state 5에서) 그만큼 중 얼마나 기다렸는가
        self.mark_check_cnt = 0
        ## (state 5에서) 그만큼 기다리는 동안 '얼마나' 발견해야 발견이라 하겠는가
        self.target_detect_cnt = rospy.get_param("target_detect_cnt")
        ## (state 5에서) 그만큼 기다리는 동안 '몇 번' 타겟을 발견했는가
        self.detected_cnt = 0

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # ON/OFF
        self.draw_contour = rospy.get_param("draw_contour")

        # other settings
        self.thruster_default = rospy.get_param("thruster_default")
        self.thruster_stop = rospy.get_param("thruster_stop")
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.span_angle = rospy.get_param("span_angle")
        self.ob_angle_range = rospy.get_param("ob_angle_range")
        self.ob_dist_range = rospy.get_param("ob_dist_range")

        # current status
        self.state = 0
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
        self.next_to_visit = (
            0  # sstate 시작을 1로할거면 1로  # 다음에 방문해야 할 스테이션 번호(state5가 false일 경우 처리하려고 만들어둠)
        )
        self.filter_queue = [0] * self.filter_queue_size

        self.distance_to_point = 0

        # controller
        cv2.namedWindow("controller")
        cv2.createTrackbar("color1 min", "controller", self.color_range[0][0], 180, lambda x: x)
        cv2.createTrackbar("color1 max", "controller", self.color_range[1][0], 180, lambda x: x)
        cv2.createTrackbar("color2 min", "controller", self.color_range[0][1], 255, lambda x: x)
        cv2.createTrackbar("color2 max", "controller", self.color_range[1][1], 255, lambda x: x)
        cv2.createTrackbar("color3 min", "controller", self.color_range[0][2], 255, lambda x: x)
        cv2.createTrackbar("color3 max", "controller", self.color_range[1][2], 255, lambda x: x)

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

    def get_trackbar_pos(self):
        """get trackbar poses and set each values"""
        self.color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
        self.color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
        self.color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
        self.color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
        self.color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
        self.color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")

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

        if not self.raw_img.size == (640 * 480 * 3):
            not_connected += "\tCamera"

        if len(not_connected) == 0:
            return True
        else:
            print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
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
            change_state = self.calc_distance(self.waypoints[0])
        elif self.state == 1:
            # 스테이션1 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[1])
        elif self.state == 2:
            # 스테이션2 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[2])
        elif self.state == 3:
            # 스테이션3 도착 여부 판단
            change_state = self.calc_distance(self.waypoints[3])
        elif self.state == 4:
            # heading 스테이션쪽인지 판단
            # TODO 정지 중간에 스테이션 쪽으로 헤딩 기울면 정지를 멈추고 바로 넘어감
            change_state = self.check_heading()
        elif self.state == 5:
            if self.mark_check_cnt >= self.target_detect_time:
                change_state = True
                self.mark_check_cnt = 0
                self.detected_cnt = 0
            else:
                change_state = False
        elif self.state == 6:
            # 도킹 완료했는지 확인
            change_state = self.check_docked()

        if change_state:
            print("")
            print("{:=^70}".format(" Change State "))
            print("")
            if self.state in [0, 1, 2, 3]:
                self.next_to_visit += 1
                if self.next_to_visit == 4:
                    # next_to_visit=4이면 전부 찾기 실패. 다시 1로 이동
                    self.next_to_visit = 1

            if self.state in [1, 2, 3]:
                self.state = 4
                # for _ in range(8):  # TODO 수정하기.
                #     self.thruster_pub.publish(1425)
                #     print("sleep")
                #     rospy.sleep(1)
                #     print("wake")
            elif self.state == 4:
                self.stop_cnt = 0
                self.state += 1
            elif self.state == 5:
                if self.target_found:
                    self.state += 1
                else:
                    self.state = self.next_to_visit
            else:
                self.state += 1

            return True
        else:
            return False

    def check_heading(self):
        # 차잇값이 ref_dir_range 보다 작으면 잘 돌린 것
        angle_to_station = self.station_dir - self.psi
        if angle_to_station >= 180:  # 왼쪽으로 회전이 더 이득
            angle_to_station = -180 + abs(angle_to_station) % 180
        elif angle_to_station <= -180:
            angle_to_station = 180 - abs(angle_to_station) % 180

        return abs(angle_to_station) <= self.ref_dir_range

    def check_target(self, return_target=False):
        self.show_window()
        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True)
        self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링
        target, self.shape_img, self.mark_area = mark_detect.detect_target(
            self.hsv_img,
            self.target_shape,
            self.mark_detect_area,
            self.target_detect_area,
            self.draw_contour,
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True

    def check_docked(self):
        if len(self.target) != 0:
            return self.target[0] >= self.arrival_target_area
        else:
            return False

    def calc_psi_desire(self):
        # 현재 heading에서 목표로 갈 때 돌려야 할 각도. 선수와 동일 선상이면 0, 우측에 있으면 +180까지 -> 180 넘을 수도 있어서 한 번 걸러줘야 함
        psi_goal = (
            math.degrees(
                math.atan2(
                    self.waypoints[self.state][1] - self.boat_y,
                    self.waypoints[self.state][0] - self.boat_x,
                )
            )
            - self.psi
        )
        self.psi_goal = rearrange_angle(psi_goal)

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
        print("")
        print("State: # {} - {}".format(str(self.state), state_str[self.state]))
        print(self.boat_x, self.boat_y)
        print("")

        if self.state == 6:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print(
                "Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area)
            )
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )
            print("")
            print(
                "mid - {:>6} = {:>11} {:->4} {:>11}".format(
                    "target", "error_pixel", ">", "error_angle"
                )
            )
            print(
                "320 - {:>6,.0f} = {:>11,.0f} {:>4} {:>11.2f} {:>9}".format(
                    self.target[1] if len(self.target) != 0 else 0,
                    320 - self.target[1] if len(self.target) != 0 else 0,
                    "",
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")

        if self.state in [0, 1, 2, 3]:
            psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            if u_servo > self.servo_middle:
                servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
            else:
                servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right
            print(
                "{:^9}   {:^8} - {:^8} = {:^8} {:->9} {:^5}".format(
                    "goal", "desire", "psi", "error", ">", "servo"
                )
            )
            print(
                "{:>9}   {:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
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
            print("{:<9} : {:6.2f} m".format("distance", self.distance_to_point))

        elif self.state == 4:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

            if u_servo > self.servo_middle:
                servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
            else:
                servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

            if self.stop_cnt >= self.stop_time:
                print("Rotating Heading >>>>")
            else:
                print("Stopping Boat >>>>>>> {:>2d} / {:>2d}".format(self.stop_cnt, self.stop_time))
            print("")
            print(
                "{:^8} - {:^8} = {:^8} {:->9} {:^5}".format("desire", "psi", "error", ">", "servo")
            )
            print(
                "{:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )

        elif self.state == 5:
            print("Target Shape : {} | Color : {}".format(self.target_shape, self.target_color))
            print(
                "Waiting..... : {:>4d} / {:>4d}".format(
                    self.mark_check_cnt, self.target_detect_time
                )
            )
            print(
                "Target Cnt   : {:>4d} / {:>4d}".format(self.detected_cnt, self.target_detect_cnt)
            )
            print("")
            print(
                "Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area)
            )

        print("")
        print("Thruster  : {}".format(u_thruster))
        print("")
        print("-" * 70)

    def show_window(self):
        self.get_trackbar_pos()
        cv2.moveWindow("controller", 100, 0)
        if self.state in [5, 6]:
            # 가로로 붙이는 버전
            raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)
            hsv_img = cv2.resize(self.hsv_img, dsize=(0, 0), fx=0.5, fy=0.5)
            hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_GRAY2BGR)
            col1 = np.vstack([raw_img, hsv_img])
            show_img = np.hstack([col1, self.shape_img])

            cv2.imshow("controller", show_img)
        else:
            # raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)
            cv2.imshow("controller", self.raw_img)


def rearrange_angle(input_angle):
    if input_angle >= 180:  # 왼쪽으로 회전이 더 이득
        output_angle = -180 + abs(input_angle) % 180
    elif input_angle <= -180:
        output_angle = 180 - abs(input_angle) % 180
    else:
        output_angle = input_angle
    return output_angle


def main():
    rospy.init_node("Docking", anonymous=True)
    docking = Docking()
    print_cnt = 0
    rate = rospy.Rate(10)

    while not docking.is_all_connected():
        rospy.sleep(0.2)
    print("\n<<<<<<<<<<<<<<<<<<< All Connected !")

    while not rospy.is_shutdown():
        docking.trajectory.append([docking.boat_x, docking.boat_y])  # 이동 경로 추가
        docking.show_window()
        change_state = docking.check_state()

        # 일부 변수 초기화
        inrange_obstacles = []
        danger_angles = []

        if docking.state in [0, 1, 2, 3]:
            docking.calc_psi_desire()

        if docking.state == 7:
            # 정지 및 끝내기
            docking.servo_pub.publish(docking.servo_middle)
            docking.thruster_pub.publish(docking.thruster_default)
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return

        elif docking.state == 0:  # 시작점으로 이동하며 장애물 회피
            inrange_obstacles, danger_angles = oa.ob_filtering(
                obstacles=docking.obstacles,
                dist_to_goal=docking.distance_to_point,
                angle_to_goal=docking.psi_goal,
                span_angle=docking.span_angle,
                angle_range=docking.ob_angle_range,
                distance_range=docking.ob_dist_range,
            )  # 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦

            # print("")
            # print("Obstacle : {:2d} / {:2d}".format(len(inrange_obstacles), len(docking.obstacles)))

            error_angle = oa.calc_desire_angle(
                danger_angles=danger_angles,
                angle_to_goal=docking.psi_goal,
                angle_range=docking.ob_angle_range,
            )
            # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            # 각도 범위가 모두 장애물이고 범위 밖에 목표지점이 있다면 psi_goal로.
            u_thruster = docking.thruster_default

        elif docking.state in [1, 2, 3]:  # 다음 스테이션으로 이동
            error_angle = docking.psi_goal
            u_thruster = docking.thruster_default

        elif docking.state == 4:  # 헤딩 돌리기
            if docking.stop_cnt >= docking.stop_time:
                # 정지 종료, 헤딩 돌리기
                u_thruster = 1600
            else:
                # 아직 멈춰 있어야 함
                docking.stop_cnt += 1
                rate.sleep()
                u_thruster = 1425
            error_angle = docking.station_dir - docking.psi  # TODO 정지에서 이상하게 움직이면 정지는 0으로 할 것.
            error_angle = rearrange_angle(error_angle)

        elif docking.state == 5:
            docking.mark_check_cnt += 1
            detected = docking.check_target()
            if detected:
                docking.detected_cnt += 1

            if docking.mark_check_cnt >= docking.target_detect_time:  # 다 기다렸는데
                if docking.detected_cnt >= docking.target_detect_cnt:  # 충분히 많이 검출되면
                    docking.target = docking.check_target(return_target=True)  # TODO 중복
                    docking.target_found = True  # 타겟 찾은 것
                else:  # 검출 횟수가 적으면
                    docking.target = [0, 0]
                    docking.target_found = False  # 타겟 못 찾은 것
            else:
                docking.target_found = False

            error_angle = docking.station_dir - docking.psi
            error_angle = rearrange_angle(error_angle)
            u_thruster = 1550  # docking.thruster_stop
            # TODO 자꾸 파도 때문에 밀려서 정지 안하긴 했으나 계속 돌아가는 것도 문제.

        elif docking.state == 6:  # 스테이션 진입
            docking.target = docking.check_target(return_target=True)
            error_angle = dock_control.pixel_to_degree(
                docking.target, docking.pixel_alpha, docking.angle_range
            )  # 양수면 오른쪽으로 가야 함
            u_thruster = docking.thruster_default

        docking.psi_desire = rearrange_angle(docking.psi + error_angle)  # 월드좌표계로 '가야 할 각도'를 계산함

        u_servo = dock_control.degree_to_servo(
            error_angle, docking.angle_range, docking.servo_range, docking.angle_alpha
        )
        # u_servo = int(
        #     filtering.moving_avg_filter(docking.filter_queue, docking.filter_queue_size, u_servo)
        # ) # 이동평균 필터링

        docking.servo_pub.publish(u_servo)
        docking.thruster_pub.publish(u_thruster)

        if print_cnt > 10:
            docking.print_status(error_angle, u_servo, u_thruster)
            print_cnt = 0
        else:
            print_cnt += 1

        all_markers = dock_visualize.visualize(
            dc=docking,
            inrange_obstacles=inrange_obstacles,
            danger_angels=danger_angles,
        )
        docking.visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()

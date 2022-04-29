#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import pymap3d as pm
import numpy as np
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tricat221.msg import Obstacle, ObstacleList

import gnss_converter as gc # src/gnss_converter.py
import point_class as pc # src/point_class.py
import rviz_viewer as rv

class Docking:
    def __init__(self) -> None:
        #==============================
        #   subscriber, publisher
        #==============================
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0) # TODO 아두이노 쪽에서 S 수정하기
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        self.star_cam = StarboardCam()

        self.detection_start_point = [0, 0]
        self.detection_end_point = [0, 0]
        self.goal_range = 1

        self.thruster_forward = 1600
        self.thruster_backward = 1200

        self.star_target_pos_thres = 200
        self.star_target_area_thres = 100
        self.bow_target_pos_range = 50
        self.bow_target_area_thres = 200

        self.state = 0   # 현재 상태
            #==============================
            #   Status
            #       0: obstacle avoidance (autonomous mode)
            #       1: docking, finding target (not arrived detection end point)
            #       2: docking, tracking target (target detected)
            #       3: docking, no target (arrived detection end point)
            #       4: docking, finished
            #==============================

        self.target_found = False

        self.next_goal = self.detection_start_point
            #==============================
            #   State: describe -> next goal
            #       0: obstacle avoidance (autonomous mode) -> 도킹 시작 지점
            #       1: docking, finding target (not arrived detection end point) -> 도킹 끝 지점
            #       2: docking, tracking target (target detected) -> 도킹 끝 지점
            #       3: docking, no target (arrived detection end point) -> 도킹 시작 지점
            #       4: docking, finished
            #==============================

        self.inrange_ob = []
        self.star_bbox = []
        self.bow_bbox = []

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수"""
        self.psi = msg.data # [degree]

    def yaw_rate_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z) # [rad/s] -> [degree/s]

    def boat_position_callback(self, msg):
        """ GPS로 측정한 배의 ENU 변환 좌표 콜백함수 """
        self.boat_x = msg.x
        self.boat_y = msg.y

    def obstacle_callback(self, msg):
        self.obstacle = msg.obstacle #[msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

    def is_all_connected(self):
        not_connected = []
        if self.heading_sub.get_num_connections() == 0:
            not_connected.append("headingCalculator")

        if self.enu_pos_sub.get_num_connections() == 0:
            not_connected.append("gnssConverter")

        if self.obstacle_sub.get_num_connections() == 0:
            not_connected.append("lidarConverter")
            
        if self.yaw_rate_sub.get_num_connections() == 0:
            not_connected.append("imu")

        if len(not_connected)==0:
            return True
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")
            print(not_connected)
            print("\n\n")
            return False

    def update_state(self):
        if self.state == 0:
            dist_to_docking_start = dist_between_pts(self.boat, self.next_goal)
                # 지금은 next_goal이 도킹 스타트 지점임
            # 방금 수행한 연산이 장애물 회피였음 -> 도킹 시작 지점부터의 거리 구함 -> 일정 거리 이하이면 전환
            if dist_to_docking_start <= self.goal_range:
                #도킹 지점 도착:
                self.state  = 1
                self.next_goal = self.detection_end_point
        elif self.state == 1:
            if self.target_found:
                self.state = 2# 방금 수행한 연산이 장애물 탐색이었음 -> mark_found가 True이면 전환
                # self.next_goal = ?????
            else:
                dist_to_docking_end = dist_between_pts(self.boat, self.next_goal)
                if dist_to_docking_end <= self.goal_range:
                    self.state = 3  # 탐색 끝지점에 왔음 -> 다 못찾음
                    self.next_goal = self.detection_start_point
                else:
                    pass    # 계속 직진
        elif self.state == 2:
            # 방금 수행한 연산이 장애물 트래킹이었음 / #탐색 끝지점 도착과는 별 상관 없음
            if self.target_found:
                area = self.bow_bbox[2] * self.bow_bbox[3]
                if area >= self.bow_target_area_thres:
                    self.state = 4 # in station
                else:
                    pass# 아직 들어가진 못함
            else:
                pass    # 큰일이다...
        elif self.state == 3:
            dist_to_docking_start = dist_between_pts(self.boat, self.next_goal)
            if dist_to_docking_start <= self.goal_range:
                #도킹 지점 도착:
                self.state  = 1
                self.next_goal = self.detection_end_point
            else:
                pass


    def avoid_ob(self):
        angle_risk = [0 for ang in range(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)]

        self.calc_ob_risk(angle_risk)
        self.calc_goal_risk(angle_risk)
        error_angle = self.calc_psi_desire(angle_risk)
        u_servo = self.ob_avoid_PID(error_angle)
        self.control_publish(u_servo, self.thruster_forward)


    def calc_ob_risk(self, angle_risk):
        self.inrange_ob = [] # 여기서 그냥 클래스 변수로 넣자
        pass

    def calc_goal_risk(self, angle_risk):
        pass

    def calc_psi_desire(self, angle_risk):
                          # 가장 위험도 낮은 각도의 인덱스
        error_angle = self.angle_risk.index(min(self.angle_risk)) * self.angle_increment + self.angle_min
                # TODO 계산식 맞는지 확인할 것 / 선박고정좌표계 기준값 -> 그대로 error_angle에 해당함
        # psi_desire = self.error_angle + self.psi   # TODO 쓸 지는 모르겠지만 일단 계산해둠

        return error_angle  #, psi_desire

    def searching_target(self):
        detected = False    # target으로 추정되는 것이 발견되었는가? self.target_found와는 다름! 확인 한 번 거쳐야 그걸로 확정 가능
        # mid_x = -1 # target mid point in pixel
        # area = -1 # contur area (bbox?)

        frame_mid = 320

        detected = self.star_cam.find_target_pos()
        print(detected)

        ### confidence check -> 테스트 후 실전에서는 줄여서 쓰자
        if detected:
            bbox = self.star_cam.bbox   # 최종 플래그가 True일 때만 유효한 값임

            # 잡히긴 잡혔으나 아직 신뢰할 정도 아닌 것들 필터링
            if bbox[0] < self.star_target_pos_thres:
                print("too corner") # TODO 발영어 어쩔거냐..
                self.target_found = False
            # elif area < self.star_target_area_thres:  -> 이미 걸러냈음
            #     print("too small")
            #     self.target_found = False
            else:
                self.star_bbox = bbox
                self.target_found = True
        else:
            self.target_found = False

        if self.target_found:
            # 회전을 얼마나 할 것인가? -> 카메라 중앙점을 기준으로 에러각을 산출한다 가정함함
            error_pixel = frame_mid - self.star_bbox[0]   # bbox = [cx, cy, w, h]
            u_servo = self.rotate_PID(error_pixel)
            self.control_publish(u_servo, self.thruster_forward)   # 잠시 정지했다가 회전했으면 좋겠는데....
        else:
            return
            


    def tracking_target(self):
        detected = False    # target으로 추정되는 것이 발견되었는가? self.target_found와는 다름! 확인 한 번 거쳐야 그걸로 확정 가능
        bbox = []   # [cx, cy, w, h] 또는 [x1, y1, x2, y2]
        mid_x = -1 # target mid point in pixel
        area = -1 # contur area (bbox?)

        frame_mid = 320

        #### 똑같은 OpenCV 부분

        self.bow_bbox = bbox


        if not detected:
            # 돌긴 돌았는데 못 찾은 것. 낭패닷...! 다시 반대로 회전해서 state=1해야 하나...
            return

        ### 여기도 실전에선 경우의 수 줄이기
        if mid_x < (frame_mid - self.bow_target_pos_range):
            print("target on right") # 중앙이 아님
        elif mid_x > (frame_mid + self.bow_target_pos_range):
            print("target on left")
        else:
            print("target in middle range")
            # 중앙이면 적게 움직이게 하는 건 일단 생략. 좌우로 많이 튀는 것 방지는 PID나 필터로 걸러내자
            
        error_pixel = frame_mid - bbox[0]   # bbox = [cx, cy, w, h]
        u_servo = self.rotate_PID(error_pixel)
        self.control_publish(u_servo, self.thruster_forward)

    def back_to_start(self):#?? 서보 중간 이거 맞ㄴ나??
        self.control_publish(self.servo_mid, self.thruster_backward)

    def ob_avoid_PID(self, error_angle):
        cp_angle = self.kp_angle * error_angle # TODO : 부호 확인하기

        self.error_sum_angle += error_angle * 0.1 # dt = rate 
        ci_angle = self.ki_angle * self.error_sum_angle
            # TODO : errorsum 초기화할 위치 선정하기

        cd_angle = - self.kd_angle * self.yaw_rate
        
        u_angle = cp_angle + ci_angle + cd_angle
        u_servo = self.servo_middle - u_angle
            # TODO : 식 맞는지 점검 필요 / 부호 주의. 왼쪽이 (-) 각도인데, 서보 각도 값은 왼쪽은 오른쪽보다 값이 커짐
        if u_servo > self.servo_left_max:
            u_servo = self.servo_left_max
        elif u_servo < self.servo_right_max:
            u_servo = self.servo_right_max

        return round(u_servo)


    def rotate_PID(self, error_pixel):
        cp_pixel = self.kp_pixel * error_pixel # TODO : 전면카메라도 계수를 공유해도 될까...?

        self.error_sum_pixel += error_pixel * 0.1 # dt = rate 
        ci_pixel = self.ki_pixel * self.error_sum_pixel
            # TODO : errorsum 초기화할 위치 선정하기

        cd_pixel = - self.kd_pixel * self.yaw_rate
        
        u_pixel = cp_pixel + ci_pixel + cd_pixel
        u_servo = self.servo_middle - u_pixel
            # TODO : 식 맞는지 점검 필요 / 부호 주의. 왼쪽이 (-) 각도인데, 서보 각도 값은 왼쪽은 오른쪽보다 값이 커짐
        if u_servo > self.servo_left_max:
            u_servo = self.servo_left_max
        elif u_servo < self.servo_right_max:
            u_servo = self.servo_right_max

        return round(u_servo)
        

    def control_publish(self, u_servo, u_thruster):
        self.trajectory.append([self.boat_x, self.boat_y]) # 현재 위치(지나온 경로의 하나가 될 점) 업데이트 # TODO 이거 왜 배열로 담아??(저장 이유?)
        self.rviz_traj.append_marker_point(self.boat_x, self.boat_y)

        self.servo_pub.publish(u_servo)
        self.thruster_pub.publish(u_thruster)

class StarboardCam:
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.starboard_callback)

        self.bridge = CvBridge()
        self.img_raw = np.empty(shape=[0])  # TODO 수정할 것
        
        self.gaussian_kernel = 5 # TODO 파라미터화하거나 트랙바로 돌릴 것
        self.morph_kernel = 5
        self.eps = 0.02
        self.min_area = 500
        self.circ_area_range = [0.5, 1.5]

        # target = rospy.get_param("target").split('-')
        #     # TODO 잘 작동하나 확인
        #     # TODO yaml 수정 / yello-모서리 수(3, 4, 5) 식으로
        str = "red-4"
        target = str.split('-')
        self.target_color = target[0]
        self.target_shape = int(target[1])
        
        cv2.namedWindow("hsv")
        cv2.createTrackbar("H lower", "hsv", 0, 180, self.trackbar_callback)
        cv2.createTrackbar("H upper", "hsv", 7, 180, self.trackbar_callback)
        cv2.createTrackbar("S lower", "hsv", 98, 255, self.trackbar_callback)
        cv2.createTrackbar("S upper", "hsv", 255, 255, self.trackbar_callback)
        cv2.createTrackbar("V lower", "hsv", 0, 255, self.trackbar_callback)
        cv2.createTrackbar("V upper", "hsv", 255, 255, self.trackbar_callback)

        self.H_bound = [0, 0]
        self.S_bound = [0, 0]
        self.V_bound = [0, 0]

        self.bbox = []

    def trackbar_callback(self, usrdata):
        pass

    def starboard_callback(self, msg):
        self.img_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def find_target_pos(self):  # 한 프레임 당 
        while not self.img_raw.size == (640 * 480 * 3):
            return

        detected = False

        # bright_adjust = self.mean_brightness() # 1. 평균 밝기로 변환
        hsv = cv2.cvtColor(self.img_raw, cv2.COLOR_BGR2HSV) # TODO: 여기 맞나?
        blur = cv2.GaussianBlur(hsv, (self.gaussian_kernel, self.gaussian_kernel), 0) # 2. 가우시안 블러
        
        self.set_HSV_value()
        mask = self.HSV_mask(blur)

        cv2.imshow("raw", self.img_raw)
        # cv2.imshow("bright_adjust", bright_adjust)
        # cv2.imshow("hsv", hsv)
        # cv2.imshow("blur", blur)

        cv2.imshow("mask", mask)

        ### 모폴로지 연산
        #################3 best 기록
        ### (1, 1) open, H upper 조금 키우기
        ### (3, 3) close, H upper 13
        ### (9, 9) close, 그대로
        ### (5, 5) open, H upper 조금 키우기

        morph_kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)

        _, contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)  # 모드 좋은 걸로 탐색
        # print("# of conturs : {}".format(len(contours)))
        
        for contour in contours:
            # print("conture size : {}".format(len(contour)))

            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * self.eps, True)

            area = cv2.contourArea(approx)
            if area < self.min_area:
                continue

            vertex_num = len(approx)
            # print("conture size : {}".format(vertex_num))

            if vertex_num == 3:
                shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
                if self.target_shape == 3:
                    shape = self.set_label(shape, approx, "Triangle")
                    detected = True
            elif vertex_num == 4:
                shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
                if self.target_shape == 4:
                    shape = self.set_label(shape, approx, "Rectangle")
                    detected = True
            else:
                _, radius = cv2.minEnclosingCircle(approx)
                ratio = radius * radius * 3.14 / area
                if ratio > self.circ_area_range[0] and ratio < self.circ_area_range[1]:
                    shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
                    if self.target_shape == 5:
                        shape = self.set_label(shape, approx, "Circle")
                        detected = True
            
        cv2.imshow("shape", shape)

        if cv2.waitKey(1) == 27:
            return  # 수정

        return True if detected else False

    def set_label(self, img, approx, label):
        drawn = cv2.drawContours(img, [approx], -1, (0, 255, 0), -1)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0))

        self.bbox = [x + w//2, y + h//2, w, h]
        print("{} : [{}, {}, {}, {}]".format(label, x + w//2, y + h//2, w, h))

        return drawn

    def mean_brightness(self):
        img = self.img_raw
        fixed = 70

        m = cv2.mean(img)
        scalar = (-int(m[0])+fixed, -int(m[1])+fixed, -int(m[2])+fixed, 0)
            # TODO 이 방법 맞나...?
        dst = cv2.add(img, scalar)

        return dst


    def set_HSV_value(self):
        self.H_bound[0] = cv2.getTrackbarPos("H lower", "hsv")
        self.H_bound[1] = cv2.getTrackbarPos("H upper", "hsv")
        self.S_bound[0] = cv2.getTrackbarPos("S lower", "hsv")
        self.S_bound[1] = cv2.getTrackbarPos("S upper", "hsv")
        self.V_bound[0] = cv2.getTrackbarPos("V lower", "hsv")
        self.V_bound[1] = cv2.getTrackbarPos("V upper", "hsv")

        
    def HSV_mask(self, img):
        lower = np.array([self.H_bound[0], self.S_bound[0], self.V_bound[0]])
        upper = np.array([self.H_bound[1], self.S_bound[1], self.V_bound[1]])
        mask = cv2.inRange(img, lower, upper)

        return mask


def dist_between_pts(p1, p2):
    # 두 점 사이의 거리를 구함
    # p1, p2는 [x, y] 좌표 가지고 있음
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])



def main():
    
    rospy.init_node('Docking', anonymous=True)
    rate = rospy.Rate(10)
    docking = Docking()



    while not docking.is_all_connected():
        rospy.sleep(0.2)

    print("\n----------All Connected----------\n")
    
    while not (docking.state == 4):
        if docking.state == 0:
            docking.avoid_ob()
        elif docking.state == 1:
            docking.searching_target()
        elif docking.state == 2:
            docking.tracking_target()
        elif docking.state == 3:
            docking.back_to_start()    # 후진해서 도킹 시작 지점으로
        elif docking.state == 4:
            break

        docking.update_state()

    rospy.spin()


if __name__=="__main__":
    main()

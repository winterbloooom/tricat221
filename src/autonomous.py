#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import sys
import pymap3d as pm
import numpy as np

from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tricat221.msg import Obstacle, ObstacleList

import perception.gnss_converter as gc # src/utils/gnss_converter.py
import utils.point_class as pc # src/utils/point_class.py
import utils.rviz_viewer as rv


class Autonomous:
    def __init__(self):
        ## subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)

        ## publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0) # TODO 아두이노 쪽에서 S 수정하기
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        ### rviz publishers
        self.rviz_angles_pub = rospy.Publisher("/angles_rviz", MarkerArray, queue_size=0)   # psi, psi_desire, angle_min, angle_max
        self.rviz_points_pub = rospy.Publisher("/points_rviz", MarkerArray, queue_size=0)   # boat, obstacle
        self.rviz_goal_pub = rospy.Publisher("/goal_rviz", Marker, queue_size=0)    # 목표점
        self.rviz_goal_txt_pub = rospy.Publisher("/goal_txt_rviz", Marker, queue_size=0)    # 목표점
        self.rviz_traj_pub = rospy.Publisher("/traj_rviz", Marker, queue_size=0)    # 지나온 경로\

        ## about goal
        self.goal_y, self.goal_x = gc.enu_convert(rospy.get_param("autonomous_goal"))
        # self.goal_x, self.goal_y = rospy.get_param('~goal_x'), rospy.get_param('~goal_y')
        self.goal_range = rospy.get_param("goal_range")
        self.distance_to_goal = 100000 #TODO 괜찮을지 확인. 처음부터 0으로 넣는다면 gps 받아오기 전부터 finished됨
        self.rviz_goal = rv.RvizMarker("goal", 11, 8, p_scale=0.2, b=1)
        self.rviz_goal.append_marker_point(self.goal_x, self.goal_y)
        self.rviz_goal_txt = rv.RvizMarker("goal_txt", 100, 9, p_scale=0, r=1, g=1, b=1)
        self.rviz_goal_txt.marker.scale.z = 0.5
        self.rviz_goal_txt.marker.pose.position = Point(self.goal_x, self.goal_y, 0)
        self.rviz_goal_txt.marker.text = "(" + str(round(self.goal_x, 2)) + ", " + str(round(self.goal_y, 2)) + ")"
        self.rviz_goal_txt.append_marker_point(self.goal_x, self.goal_y)

        ## about position
        self.boat_x, self.boat_y = 0, 0
        self.trajectory = []
        self.rviz_traj = rv.RvizMarker("traj", 8, 8, p_scale=0.1, g=1)
        # self.rviz_traj.append_marker_point(self.boat_x, self.boat_y)
        
        ## about angles
        self.angle_min = rospy.get_param("angle_min") # 목표 각도 후보 최솟값
        self.angle_max = rospy.get_param("angle_max") # 목표 각도 후보 최댓값
        self.angle_increment = rospy.get_param("angle_increment") # 각도 계산할 단위각
        self.span_angle = rospy.get_param("span_angle")
        self.angle_risk = [0 for ang in range(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)]
            # 목표각 탐색 범위임. TODO : 수정 필요할 수도!
        self.psi = 0
        self.psi_desire = 0
        self.psi_goal = math.degrees(math.atan2(self.goal_y, self.goal_x))    # 한 번 넣으면 변하지 않는 값임
        self.error_angle = 0

        ## coefficients for calculations
        self.ob_exist_coefficient = rospy.get_param("ob_exist_coefficient")
        self.ob_near_coefficient = rospy.get_param("ob_near_coefficient")
        self.goal_risk_range = rospy.get_param("goal_risk_range")
        self.goal_orient_coefficient = rospy.get_param("goal_orient_coefficient")
        
        ## about obstacles
        self.obstacle = []
        self.inrange_obstacle = []

        ## variables for PID control
        self.yaw_rate = 0 # z축 각속도 [degree/s]
        self.error_sum_angle = 0
        self.kp_angle = rospy.get_param("kp_angle")
        self.ki_angle = rospy.get_param("ki_angle")
        self.kd_angle = rospy.get_param("kd_angle")

        ## servo motor range
        self.servo_middle = rospy.get_param("servo_middle")
        self.servo_left_max = rospy.get_param("servo_left_max")
        self.servo_right_max = rospy.get_param("servo_right_max")

        ## thruster range
        self.thruster_max = rospy.get_param("thruster_max")
        self.thruster_min = rospy.get_param("thruster_min")

        ## for report
        self.cnt = 0 # print 출력 속도 조절 위한 타이머
        
        ## first call functions
        # self.calc_angle_to_goal()
        self.calc_distance_to_goal()


    ######################## Callback 함수들 ########################
    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수"""
        self.psi = msg.data # [degree]

    def yaw_rate_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z) # [rad/s] -> [degree/s]

    def boat_position_callback(self, msg):
        """ GPS로 측정한 배의 ENU 변환 좌표 콜백함수 """
        # self.boat_y = msg.x
        # self.boat_x = msg.y
        self.boat_y = msg.x
        self.boat_x = msg.y

    def obstacle_callback(self, msg):
        self.obstacle = msg.obstacle #[msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]
     
    ######################## 업데이트 및 체크 관련 함수들 ########################
    def calc_distance_to_goal(self):
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)


    def arrival_check(self):
        self.calc_distance_to_goal() #목적지까지 거리 다시 계산
        if self.distance_to_goal <= self.goal_range:
            return True
        else:
            return False


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


    ######################## 경로 계산 관련 함수들 ########################
    def calc_angle_risk(self):
        """ 각도 위험도 구하기 """
        self.angle_risk = [0 for ang in range(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)]
        self.calc_ob_risk()  # (1) 장애물이 존재하는 각도 (2) 지금 배와 가까운 곳에 존재하는 장애물의 각도 등을 기준으로 점수 메기기 
        self.calc_goal_risk() # (3) 목표점에 가까운 각도는 적은 위험 멀면 큰 위험 

        self.calc_psi_desire() # 다음 목표각을 설정함


    def calc_ob_risk(self):
        self.inrange_obstacle = []

        for ob in self.obstacle:
            begin_ang = math.degrees(math.atan2(ob.begin.y, -ob.begin.x)) # 장애물 블럭 시작점
            end_ang = math.degrees(math.atan2(ob.end.y, -ob.end.x))   # 장애물 블럭 끝점

            middle_x = -(ob.begin.x + ob.end.x) / 2  # 장애물 블럭 중점
            middle_y = (ob.begin.y + ob.end.y) / 2
            middle_ang = round(math.degrees(math.atan2(middle_y, middle_x))) # TODO increment를 1로 했을 때를 가정해 round를 했음 소숫점까지 내려가면 모르겠다..

            if middle_ang < self.angle_min or middle_ang > self.angle_max:
                continue  # min_angle, max_angle 벗어난 것 처리함. TODO 잘 들어가는지 확인
            else:
                self.inrange_obstacle.append(ob)    # 계산 범위에 있는 장애물임 # TODO 나중에는 Rviz 상에서 보도록 span도 추가해서보자

            if (begin_ang - self.span_angle) > self.angle_min:
                start_ang = round(begin_ang - self.span_angle)
            else:
                start_ang = self.angle_min
            
            if (end_ang + self.span_angle) < self.angle_max:
                fin_ang = round(end_ang + self.span_angle)
            else:
                fin_ang = self.angle_max
            
            start_ang_idx = int((start_ang - self.angle_min) / self.angle_increment)
            fin_ang_idx = int((fin_ang - self.angle_min) / self.angle_increment)
            middle_ang_idx = int((start_ang_idx + fin_ang_idx) / 2) # 장애물이 계산 범위 넘어가 있어 start_ang / fin_ang이 바뀌었더라도 탐지범위 내 있는 크기의 중간으로 삼음

            for idx in range(start_ang_idx, fin_ang_idx + 1, self.angle_increment):
                self.angle_risk[idx] += self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

            dist = math.sqrt(pow(self.boat_x - middle_x, 2.0) + pow(self.boat_y - middle_y, 2.0))   # 장애물 중점에서 배까지 거리
            self.angle_risk[middle_ang_idx] -= dist * self.ob_near_coefficient  # 거리에 비례해 곱해줌. 때문에 해당 계수는 1 이하여야함



    def calc_goal_risk(self): # 목표점에 가까워야 하니까 위험도는 낮아짐.
        error_goal_ang = self.psi_goal - self.psi # 선박고정좌표계 기준 도착지까지 각도

        if self.angle_min <= error_goal_ang <= self.angle_max: # 계산 범위 안에 목표점이 존재함
            for idx in range(len(self.angle_risk)):
                ang = idx * self.angle_increment + self.angle_min
                delta_ang = abs(error_goal_ang - ang) # TODO 값 확인하기
                if delta_ang <= self.goal_risk_range:
                    self.angle_risk[idx] += 1 * self.goal_orient_coefficient   # 해당 각도가 목표점과 많이 떨어져 있지 않음. 값을 조금만 더해줌
                elif delta_ang <= (self.goal_risk_range * 2):
                    self.angle_risk[idx] += 2 * self.goal_orient_coefficient
                elif delta_ang <= (self.goal_risk_range * 3):
                    self.angle_risk[idx] += 3 * self.goal_orient_coefficient
                else:
                    self.angle_risk[idx] += 4 * self.goal_orient_coefficient
        elif self.angle_min > error_goal_ang: # 목적지가 탐색 범위를 벗어난 좌현에 위치 -> TODO 일단 중심 각도를 기준으로 우쪽에 모두 점수 부여. 반대쪽이니까 우현에 안 가도록 위험도 높임
            for idx in range(len(self.angle_risk)//2, len(self.angle_risk)):
                self.angle_risk[idx] += 2 * self.goal_orient_coefficient # TODO 2를 곱하는 게 맞을까?
        else:   # 목적지가 탐색 범위를 벗어난 우현에 위치(ex. 출발 지점) -> TODO 일단 중심 각도를 기준으로 왼쪽에 모두 점수 부여
            for idx in range(0, len(self.angle_risk)//2 + 1):
                self.angle_risk[idx] += 2 * self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?

        # if self.angle_min <= self.psi_goal <= self.angle_max: # 목표각 탐색 범위 안에 목표점이 존재함
        #     for idx in range(len(self.angle_risk)):
        #         ang = idx * self.angle_increment + self.angle_min
        #         delta_ang = abs(self.psi - ang) # TODO 값 확인하기
        #         if delta_ang <= self.goal_risk_range:
        #             continue
        #         elif delta_ang <= (self.goal_risk_range * 2):
        #             self.angle_risk[idx] -= 1*self.goal_orient_coefficient
        #         elif delta_ang <= (self.goal_risk_range * 3):
        #             self.angle_risk[idx] -= 2*self.goal_orient_coefficient
        #         else:
        #             self.angle_risk[idx] -= 3*self.goal_orient_coefficient
        # elif self.angle_min > self.psi_goal: # 목적지가 탐색 범위를 벗어난 좌현에 위치 -> TODO 일단 중심 각도를 기준으로 왼쪽에 모두 점수 부여
        #     for idx in range(len(self.angle_risk)//2):
        #         self.angle_risk[idx] -= 1*self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?
        # else:   # 목적지가 탐색 범위를 벗어난 우현에 위치(ex. 출발 지점) -> TODO 일단 중심 각도를 기준으로 오른쪽에 모두 점수 부여
        #     for idx in range(len(self.angle_risk)//2, len(self.angle_risk) + 1):
        #         self.angle_risk[idx] -= 1*self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?


    def calc_psi_desire(self):
                          # 가장 위험도 낮은 각도의 인덱스
        self.error_angle = self.angle_risk.index(min(self.angle_risk)) * self.angle_increment + self.angle_min
                # TODO 계산식 맞는지 확인할 것 / 선박고정좌표계 기준값 -> 그대로 error_angle에 해당함
        self.psi_desire = self.error_angle + self.psi   # TODO 쓸 지는 모르겠지만 일단 계산해둠


    def error_angle_PID(self):
        cp_angle = self.kp_angle * self.error_angle # TODO : 부호 확인하기

        self.error_sum_angle += self.error_angle * 0.1 # dt = rate 
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

        # print("cp_angle : {} / ci_angle : {} / cd_angle : {}".format(cp_angle, ci_angle, cd_angle))
        # print("u_angle : {} / u_servo : {}".format(u_angle, u_servo))

        return round(u_servo)


    def control_publish(self):
        # 에러각 계산 -> PID로
        # self.calc_psi_desire() # TODO 다시 들어가 있네...
        self.u_servo = self.error_angle_PID()

        # 남은 거리 계산 -> PID로 / TODO  속도는 동일하게 간다고 하고 PID 사용 유보
        # self.calc_distance_to_goal()
        # self.u_thruster = self.distance_PID()

        self.trajectory.append([self.boat_x, self.boat_y]) # 현재 위치(지나온 경로의 하나가 될 점) 업데이트 # TODO 이거 왜 배열로 담아??(저장 이유?)
        self.rviz_traj.append_marker_point(self.boat_x, self.boat_y)

        self.servo_pub.publish(self.u_servo)
        self.thruster_pub.publish() # TODO thruster도 PID 할 필요 없다면 굳이 여기 위치시킬 필요 있나?


    def print_state(self):
        if self.cnt < 10:
            self.cnt += 1
            return

        print("-"*20)
        print("")   # TODO : 자릿수 맞추기 / 더 출력해봐야 할 것 있나?
        print("Boat loc : [{0}, {1}]".format(self.boat_x, self.boat_y))
        print("Goal : [{0}, {1}]".format(self.goal_x, self.goal_y))
        print("Dist. to Goal: {0} m | Ang. to Goal: {1}".format(self.distance_to_goal, self.psi_goal))
        print("index of min angle risk: {0}".format(self.angle_risk.index(min(self.angle_risk))))
        # print(self.angle_risk)
        print("psi(heading): {0}, psi_desire(wanna go): {1}".format(self.psi, self.psi_desire))
        print("Error Angle(psi-desire): {0} deg | Servo : {1}".format(self.error_angle, self.u_servo))        
        print("")

        self.cnt = 0


    def view_rviz(self):
        rviz_points_arr = MarkerArray()
        rviz_ang_arr = MarkerArray()

        heading = rv.RvizMarker("heading", 4, 0, a_x_scale=0.2, a_y_scale=0.4, g=0.55, b=0.55)
        heading.append_marker_point(self.boat_x, self.boat_y)
        heading.append_marker_point(2 * math.cos(math.radians(self.psi)) + self.boat_x,\
                                    2 * math.sin(math.radians(self.psi)) + self.boat_y)

        psi_desire = rv.RvizMarker("psi_d", 5, 0, a_x_scale=0.2, a_y_scale=0.4, r=0.39, g=0.58, b=0.93)
        psi_desire.append_marker_point(self.boat_x, self.boat_y)
        psi_desire.append_marker_point(2 * math.cos(math.radians(self.psi_desire)) + self.boat_x,\
                                    2 * math.sin(math.radians(self.psi_desire)) + self.boat_y)
        
        psi_desire_txt = rv.RvizMarker("psi_d_txt", 13, 9, r=1, g=1, b=1)
        psi_desire_txt.marker.scale.z = 0.5
        psi_desire_txt.marker.text = "psi_desire"
        psi_desire_txt.marker.pose.position = Point(2.2 * math.cos(math.radians(self.psi_desire)) + self.boat_x,\
                                    2.2 * math.sin(math.radians(self.psi_desire)) + self.boat_y, 0)

        boat = rv.RvizMarker("boat", 6, 8, p_scale=0.2, r=1)
        boat.append_marker_point(self.boat_x, self.boat_y)

        ob_mark = rv.RvizMarker("obstacles", 7, 5, p_scale=0.05, r=1, b=1)
        for ob in self.obstacle:
            """
            (우리는 x, y가 바뀌어야 함)(부호도 반대) -> 그러니까 똑같네???
            ycos + xsin = y'
            -ysin + xcos = x'
            """
            # begin_x = self.boat_x + ob.begin.x * math.cos(math.radians(self.psi)) - ob.begin.y * math.sin(math.radians(self.psi))
            # begin_y = self.boat_y + ob.begin.x * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))
            # end_x = self.boat_x + ob.end.x * math.cos(math.radians(self.psi)) - ob.end.y * math.sin(math.radians(self.psi))
            # end_y = self.boat_y + ob.end.x * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

            # ob_mark.append_marker_point(begin_x, begin_y)
            # ob_mark.append_marker_point(end_x, end_y)

            begin_x = self.boat_x + (-ob.begin.x) * math.cos(math.radians(self.psi)) - ob.begin.y * math.sin(math.radians(self.psi))
            begin_y = self.boat_y + (-ob.begin.x) * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))
            end_x = self.boat_x + (-ob.end.x) * math.cos(math.radians(self.psi)) - ob.end.y * math.sin(math.radians(self.psi))
            end_y = self.boat_y + (-ob.end.x) * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

            ob_mark.append_marker_point(begin_x, begin_y)
            ob_mark.append_marker_point(end_x, end_y)

            # ob_mark.append_marker_point(self.boat_x - ob.begin.x, self.boat_y + ob.begin.y)
            # ob_mark.append_marker_point(self.boat_x - ob.end.x, self.boat_y + ob.end.y)

        inrange_ob_mark = rv.RvizMarker("inrange_obstacles", 12, 5, p_scale=0.1, r=1, g=1)
        for ob in self.inrange_obstacle:
            # begin_x = self.boat_x + ob.begin.x * math.cos(math.radians(self.psi)) - ob.begin.y * math.sin(math.radians(self.psi))
            # begin_y = self.boat_y + ob.begin.x * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))
            # end_x = self.boat_x + ob.end.x * math.cos(math.radians(self.psi)) - ob.end.y * math.sin(math.radians(self.psi))
            # end_y = self.boat_y + ob.end.x * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

            begin_x = self.boat_x + (-ob.begin.x) * math.cos(math.radians(self.psi)) - ob.begin.y * math.sin(math.radians(self.psi))
            begin_y = self.boat_y + (-ob.begin.x) * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))
            end_x = self.boat_x + (-ob.end.x) * math.cos(math.radians(self.psi)) - ob.end.y * math.sin(math.radians(self.psi))
            end_y = self.boat_y + (-ob.end.x) * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

            inrange_ob_mark.append_marker_point(begin_x, begin_y)
            inrange_ob_mark.append_marker_point(end_x, end_y)

            # inrange_ob_mark.append_marker_point(self.boat_x - ob.begin.x, self.boat_y + ob.begin.y)
            # inrange_ob_mark.append_marker_point(self.boat_x - ob.end.x, self.boat_y + ob.end.y)

        detect_start = rv.RvizMarker("detect_start", 9, 5, p_scale=0.06, r=0.5)
        detect_start.append_marker_point(self.boat_x, self.boat_y)
        detect_start.append_marker_point(3 * math.cos(math.radians(self.psi + self.angle_min)) + self.boat_x,\
                                    3 * math.sin(math.radians(self.psi + self.angle_min)) + self.boat_y)

        detect_end = rv.RvizMarker("detect_end", 10, 5, p_scale=0.06, g=0.5)
        detect_end.append_marker_point(self.boat_x, self.boat_y)
        detect_end.append_marker_point(3 * math.cos(math.radians(self.psi + self.angle_max)) + self.boat_x,\
                                    3 * math.sin(math.radians(self.psi + self.angle_max)) + self.boat_y)
  
        self.rviz_traj_pub.publish(self.rviz_traj.marker)
        self.rviz_goal_pub.publish(self.rviz_goal.marker)
        self.rviz_goal_txt_pub.publish(self.rviz_goal_txt.marker)

        rviz_points_arr.markers.append(boat.marker)
        rviz_points_arr.markers.append(ob_mark.marker)
        rviz_points_arr.markers.append(inrange_ob_mark.marker)
        self.rviz_points_pub.publish(rviz_points_arr)
        
        rviz_ang_arr.markers.append(heading.marker)
        rviz_ang_arr.markers.append(psi_desire.marker)
        rviz_ang_arr.markers.append(psi_desire_txt.marker)
        rviz_ang_arr.markers.append(detect_start.marker)
        rviz_ang_arr.markers.append(detect_end.marker)
        self.rviz_angles_pub.publish(rviz_ang_arr)


def main():
    rospy.init_node('autonomous', anonymous=False)

    autonomous = Autonomous()
    rate = rospy.Rate(10)

    while not autonomous.is_all_connected():
        rospy.sleep(0.2)

    while not rospy.is_shutdown():
        if autonomous.arrival_check(): # 최종 목적지에 도착함
            autonomous.servo_pub.publish(autonomous.servo_middle)
            autonomous.thruster_pub.publish(0) # TODO: 정지값 넣어주기
            print("-"*20)
            print("Finished!")
            return
        else:
            autonomous.calc_angle_risk()
            autonomous.control_publish()

        autonomous.print_state()
        autonomous.view_rviz()
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()



####################################3 Trash
    # def calc_ob_risk(self):
        #     """
        #     회전변환행렬 이용. [heading 방향으로 0도로 하는 Xb-Yb축] -> [진북을 0도로 하는 Xo-Yo 축]
        #     | cos(a)  -sin(a) | | x | | x' |
        #     |                 |.|   |=|    |
        #     | sin(a)   cos(a) | | y | | y' |
        #     """
        #     # TODO msg, 계산 잘 들어가는지 확인
        #     begin = pc.Point.empty_point()
        #     begin.x = ob.begin.x * math.cos(math.radians(self.psi)) - ob.begin.x * math.sin(math.radians(self.psi))
        #     begin.y = ob.begin.y * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))

        #     end = pc.Point.empty_point()
        #     end.x = ob.end.x * math.cos(math.radians(self.psi)) - ob.end.x * math.sin(math.radians(self.psi))
        #     end.y = ob.end.y * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

        #     middle = (begin + end) / 2

        #     start_ang = round(begin.polar_angle_deg() - self.span_angle) # TODO 부호 합당한가 확인. lidar converter의 반시계 방향 건 참고해서
        #     end_ang = round(end.polar_angle_deg() + self.span_angle) # TODO round 올림 어디서 할 건지 결정
        #     middle_ang = round(middle.polar_angle_deg())

        #     if end_ang < self.angle_min or start_ang > self.angle_max:
        #         continue  # min_angle, max_angle 벗어난 것 처리함. TODO 잘 들어가는지 확인 / del ob, del obstacle[obstacle.index(ob)] 안됨

        #     start_ang_idx = int((start_ang - self.angle_min) / self.angle_increment) if (start_ang > self.angle_min) else 0 # int 안 씌워도 되는지? # self.angle_risk.find(start_ang)
        #     end_ang_idx = int((end_ang - self.angle_min) / self.angle_increment) if (end_ang < self.angle_max) else len(self.angle_risk) - 1
        #     if middle_ang < self.angle_min:
        #         middle_ang_idx = 0
        #     elif middle_ang > self.angle_max:
        #         middle_ang_idx = len(self.angle_risk) - 1
        #     else:
        #         middle_ang_idx = int((middle_ang - self.angle_min) / self.angle_increment)
            
        #     # if start_ang_idx == -1 or end_ang_idx == -1 or middle_ang_idx == -1: # TODO 내가 해놓고도 이거 어떻게 동작하는지 까먹음.. 실화?
        #     #     print("Error!")
        #     # else:
        #     #     for idx in range(start_ang_idx, end_ang_idx, self.angle_increment):
        #     #         self.angle_risk[idx] += 1*self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

        #     for idx in range(start_ang_idx, end_ang_idx, self.angle_increment):
        #         self.angle_risk[idx] += 1*self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

        #     dist = middle.dist_btw_points2(self.boat_x, self.boat_y)
        #     dist_to_obstacles.append([dist, middle_ang_idx])
        
        # dist_to_obstacles.sort() # 거리를 오름차순 정렬. 가장 가까운 장애물부터.
        # for d in dist_to_obstacles:
        #     self.angle_risk[d[1]] += 1*self.ob_near_coefficient # TODO 잘 작동하는지 확인할 것
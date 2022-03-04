#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import pymap3d as pm

from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Vector3
from tricat221.msg import Obstacle, ObstacleList
from visualization_msgs.msg import Marker, MarkerArray

import gnss_converter as gc # src/gnss_converter.py
import point_class as pc # src/point_class.py
import rviz_viewer as rv


class Autonomous:
    def __init__(self):
        ## sub, pub
        heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0) # TODO 아두이노 쪽에서 S 수정하기
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        ### 계속 변하는 값 rviz pub
        self.rviz_angles_pub = rospy.Publisher("/angles_rviz", MarkerArray, queue_size=0)   # heading, psi_desire(목표까지의 에러각), 지금 서보로 돌릴 각
        self.rviz_points_pub = rospy.Publisher("/points_rviz", MarkerArray, queue_size=0)   # 현재 배 위치, 장애물 위치
        ### 변하지 않는 값, 누적값 rviz pub
        self.rviz_goal_pub = rospy.Publisher("/goal_rviz", Marker, queue_size=0)    # 목표점
        self.rviz_traj_pub = rospy.Publisher("/traj_rviz", Marker, queue_size=0)    # 지나온 경로\

        ## 파라미터 및 변수
        self.goal_x, self.goal_y = 3, 5 #gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.goal_range = rospy.get_param("goal_range")

        ### rviz module test
        self.rviz_goal = rv.RvizMarker("goal", 11, 8, 0.2, 0, 0, 1)
        self.rviz_goal.append_marker_point(self.goal_x, self.goal_y)
        
        self.angle_min = rospy.get_param("angle_min") # 목표 각도 후보 최솟값
        self.angle_max = rospy.get_param("angle_max") # 목표 각도 후보 최댓값
        self.angle_increment = rospy.get_param("angle_increment") # 각도 계산할 단위각
        self.span_angle = rospy.get_param("span_angle")
        self.angle_risk = [0 for ang in range(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)]
            # 목표각 탐색 범위임. TODO : 수정 필요할 수도!
        self.goal_risk_range = rospy.get_param("goal_risk_range")
        self.ob_exist_coefficient = rospy.get_param("ob_exist_coefficient")
        self.ob_near_coefficient = rospy.get_param("ob_near_coefficient")
        self.goal_orient_coefficient = rospy.get_param("goal_orient_coefficient")

        self.error_sum_angle = 0
        self.kp_angle = rospy.get_param("kp_angle")
        self.ki_angle = rospy.get_param("ki_angle")
        self.kd_angle = rospy.get_param("kd_angle")

        self.servo_middle = rospy.get_param("servo_middle")
        self.servo_left_max = rospy.get_param("servo_left_max")
        self.servo_right_max = rospy.get_param("servo_right_max")

        self.thruster_max = rospy.get_param("thruster_max")
        self.thruster_min = rospy.get_param("thruster_min")

        self.boat_x = 0
        self.boat_y = 0
        self.trajectory = []

        self.yaw_rate = 0 # z축 각속도 [degree/s]

        self.psi = 0
        self.psi_desire = 0
        self.psi_goal = 0
        self.calc_angle_to_goal()

        self.obstacle = []

        self.distance_to_goal = 100000 #TODO 괜찮을지 확인. 처음부터 0으로 넣는다면 gps 받아오기 전부터 finished됨
        self.calc_distance_to_goal()

        self.error_angle = 0

        self.cnt = 0 # print 출력 속도 조절 위한 타이머

        self.rviz_traj = rv.RvizMarker("traj", 8, 8, 0.1, 0, 1, 0)
        self.rviz_traj.append_marker_point(self.boat_x, self.boat_y)
        

    # IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수
    def heading_callback(self, msg):
        self.psi = msg.data # [degree]


    def yaw_rate_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z) # [rad/s] -> [degree/s]
    

    # GPS로 측정한 배의 ENU 변환 좌표 콜백함수
    def boat_position_callback(self, msg):
        self.boat_x = msg.x
        self.boat_y = msg.y


    def obstacle_callback(self, msg):
        self.obstacle = msg.obstacle #[msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]


    def calc_distance_to_goal(self):
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)


    def calc_angle_to_goal(self):
        self.psi_goal = math.atan2(self.goal_y, self.goal_x)


    def arrival_check(self):
        self.calc_distance_to_goal() #목적지까지 거리 다시 계산
        if self.distance_to_goal <= self.goal_range:
            return True
        else:
            return False


    # 각도 위험도 구하기
    def calc_angle_risk(self):
        self.angle_risk = [0 for ang in range(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)]
        self.calc_ob_risk()  # (1) 장애물이 존재하는 각도 (2) 지금 배와 가까운 곳에 존재하는 장애물의 각도 등을 기준으로 점수 메기기 
        self.calc_goal_risk() # (3) 목표점에 가까운 각도는 적은 위험 멀면 큰 위험 

        self.calc_psi_desire() # 다음 목표각을 설정함


    def calc_ob_risk(self):
        dist_to_obstacles = [] # [장애물 중점까지 거리, 각도 인덱스]의 배열

        for ob in self.obstacle:
            """
            회전변환행렬 이용. [heading 방향으로 0도로 하는 Xb-Yb축] -> [진북을 0도로 하는 Xo-Yo 축]
            | cos(a)  -sin(a) | | x | | x' |
            |                 |.|   |=|    |
            | sin(a)   cos(a) | | y | | y' |
            """
            # TODO msg, 계산 잘 들어가는지 확인
            begin = pc.Point.empty_point()
            begin.x = ob.begin.x * math.cos(math.radians(self.psi)) - ob.begin.x * math.sin(math.radians(self.psi))
            begin.y = ob.begin.y * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))

            end = pc.Point.empty_point()
            end.x = ob.end.x * math.cos(math.radians(self.psi)) - ob.end.x * math.sin(math.radians(self.psi))
            end.y = ob.end.y * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))

            middle = (begin + end) / 2

            start_ang = round(begin.polar_angle_deg() - self.span_angle) # TODO 부호 합당한가 확인. lidar converter의 반시계 방향 건 참고해서
            end_ang = round(end.polar_angle_deg() + self.span_angle) # TODO round 올림 어디서 할 건지 결정
            middle_ang = round(middle.polar_angle_deg())

            if end_ang < self.angle_min or start_ang > self.angle_max:
                continue  # min_angle, max_angle 벗어난 것 처리함. TODO 잘 들어가는지 확인 / del ob, del obstacle[obstacle.index(ob)] 안됨

            start_ang_idx = int((start_ang - self.angle_min) / self.angle_increment) if (start_ang > self.angle_min) else 0 # int 안 씌워도 되는지? # self.angle_risk.find(start_ang)
            end_ang_idx = int((end_ang - self.angle_min) / self.angle_increment) if (end_ang < self.angle_max) else len(self.angle_risk) - 1
            if middle_ang < self.angle_min:
                middle_ang_idx = 0
            elif middle_ang > self.angle_max:
                middle_ang_idx = len(self.angle_risk) - 1
            else:
                middle_ang_idx = int((middle_ang - self.angle_min) / self.angle_increment)
            
            # if start_ang_idx == -1 or end_ang_idx == -1 or middle_ang_idx == -1: # TODO 내가 해놓고도 이거 어떻게 동작하는지 까먹음.. 실화?
            #     print("Error!")
            # else:
            #     for idx in range(start_ang_idx, end_ang_idx, self.angle_increment):
            #         self.angle_risk[idx] += 1*self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

            for idx in range(start_ang_idx, end_ang_idx, self.angle_increment):
                self.angle_risk[idx] += 1*self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

            dist = middle.dist_btw_points2(self.boat_x, self.boat_y)
            dist_to_obstacles.append([dist, middle_ang_idx])
        
        dist_to_obstacles.sort() # 거리를 오름차순 정렬. 가장 가까운 장애물부터.
        for d in dist_to_obstacles:
            self.angle_risk[d[1]] += 1*self.ob_near_coefficient # TODO 잘 작동하는지 확인할 것


    def calc_goal_risk(self): # 목표점에 가까워야 하니까 위험도는 낮아짐. 그래서 - 연산
        if self.angle_min <= self.psi_goal <= self.angle_max: # 목표각 탐색 범위 안에 목표점이 존재함
            for idx in range(len(self.angle_risk)):
                ang = idx * self.angle_increment + self.angle_min
                delta_ang = abs(self.psi - ang) # TODO 값 확인하기
                if delta_ang <= self.goal_risk_range:
                    continue
                elif delta_ang <= (self.goal_risk_range * 2):
                    self.angle_risk[idx] -= 1*self.goal_orient_coefficient
                elif delta_ang <= (self.goal_risk_range * 3):
                    self.angle_risk[idx] -= 2*self.goal_orient_coefficient
                else:
                    self.angle_risk[idx] -= 3*self.goal_orient_coefficient
        elif self.angle_min > self.psi_goal: # 목적지가 탐색 범위를 벗어난 좌현에 위치 -> TODO 일단 중심 각도를 기준으로 왼쪽에 모두 점수 부여
            for idx in range(len(self.angle_risk)//2):
                self.angle_risk[idx] -= 1*self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?
        else:   # 목적지가 탐색 범위를 벗어난 우현에 위치(ex. 출발 지점) -> TODO 일단 중심 각도를 기준으로 오른쪽에 모두 점수 부여
            for idx in range(len(self.angle_risk)//2, len(self.angle_risk) + 1):
                self.angle_risk[idx] -= 1*self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?


    def calc_psi_desire(self):
                          # 가장 위험도 낮은 각도의 인덱스
        self.psi_desire = self.angle_risk.index(min(self.angle_risk)) * self.angle_increment + self.angle_min # TODO 계산식 맞는지 확인할 것
        self.error_angle = self.psi_desire - self.psi


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

        return round(u_servo)


    def control_publish(self):
        # 에러각 계산 -> PID로
        self.calc_psi_desire()
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
        print("Error Angle  : {0} deg | Servo   : {1}".format(self.error_angle, self.u_servo))
        print("Dist. to Goal: {0} m".format(self.distance_to_goal))
        print("index of min angle risk: {0}".format(self.angle_risk.index(min(self.angle_risk))))
        print(self.angle_risk)
        print("psi: {0}, psi_desire: {1}".format(self.psi, self.psi_desire))
        print("")

        self.cnt = 0


    def view_rviz(self):
        rviz_points_arr = MarkerArray()
        rviz_ang_arr = MarkerArray()

        heading = rv.RvizMarker("heading", 4, 0, 0, 0.5, 0, 0.5)
        heading.append_marker_point(self.boat_x, self.boat_y)
        heading.append_marker_point(2 * math.cos(math.radians(self.psi)) + self.boat_x,\
                                    2 * math.sin(math.radians(self.psi)) + self.boat_y)

        psi_desire = rv.RvizMarker("psi_d", 5, 0, 0, 1, 0.4, 0.7)
        psi_desire.append_marker_point(self.boat_x, self.boat_y)
        psi_desire.append_marker_point(2 * math.cos(math.radians(self.psi_desire)) + self.boat_x,\
                                    2 * math.sin(math.radians(self.psi_desire)) + self.boat_y)

        boat = rv.RvizMarker("boat", 6, 8, 0.2, 1, 0, 0)
        boat.append_marker_point(self.boat_x, self.boat_y)

        ob_mark = rv.RvizMarker("obstacles", 7, 5, 0.05, 1, 1, 0)
        for ob in self.obstacle:
            begin_x = self.boat_x + ob.begin.x * math.cos(math.radians(self.psi)) - ob.begin.x * math.sin(math.radians(self.psi))
            begin_y = self.boat_y + ob.begin.y * math.sin(math.radians(self.psi)) + ob.begin.y * math.cos(math.radians(self.psi))
            end_x = self.boat_x + ob.end.x * math.cos(math.radians(self.psi)) - ob.end.x * math.sin(math.radians(self.psi))
            end_y = self.boat_y + ob.end.y * math.sin(math.radians(self.psi)) + ob.end.y * math.cos(math.radians(self.psi))
            ob_mark.append_marker_point(begin_x, begin_y)
            ob_mark.append_marker_point(end_x, end_y)

        detect_start = rv.RvizMarker("detect_start", 9, 5, 0.03, 0.5, 0, 0)
        detect_start.append_marker_point(self.boat_x, self.boat_y)
        detect_start.append_marker_point(3 * math.cos(math.radians(self.angle_min)) + self.boat_x,\
                                    3 * math.sin(math.radians(self.angle_min)) + self.boat_y)

        detect_end = rv.RvizMarker("detect_end", 10, 5, 0.03, 0, 0.5, 0)
        detect_end.append_marker_point(self.boat_x, self.boat_y)
        detect_end.append_marker_point(3 * math.cos(math.radians(self.angle_max)) + self.boat_x,\
                                    3 * math.sin(math.radians(self.angle_max)) + self.boat_y)
  
        self.rviz_traj_pub.publish(self.rviz_traj.marker)
        self.rviz_goal_pub.publish(self.rviz_goal.marker)

        rviz_points_arr.markers.append(boat.marker)
        rviz_points_arr.markers.append(ob_mark.marker)
        self.rviz_points_pub.publish(rviz_points_arr)
        
        rviz_ang_arr.markers.append(heading.marker)
        rviz_ang_arr.markers.append(psi_desire.marker)
        rviz_ang_arr.markers.append(detect_start.marker)
        rviz_ang_arr.markers.append(detect_end.marker)
        self.rviz_angles_pub.publish(rviz_ang_arr)

    
    def is_all_connected(self):
        not_connected = []
        if heading_sub.get_num_connections() == 0:
            not_connected.append("headingCalculator")

        if enu_pos_sub.get_num_connections() == 0
            not_connected.append("gnssConverter")

        if obstacle_sub.get_num_connections() == 0:
            not_connected.append("lidarConverter")
            
        if yaw_rate_sub.get_num_connections() == 0:
            not_connected.append("imu")

        if len(not_connected)==0:
            return False
        else:
            print("\n\n----------...NOT CONNECTED YET...----------")
            print(not_connected)
            print("\n\n")
            rospy.sleep(0.1)


def main():
    rospy.init_node('autonomous', anonymous=False)

    autonomous = Autonomous()
    rate = rospy.Rate(10)

    while autonomous.is_all_connected():
        pass

    while not rospy.is_shutdown():
        # autonomous.calc_angle_risk()
        # autonomous.control_publish()

        # TODO if, else 문 주석 해제한 버전으로 쓰면 처음부터 Finished가 나옴. 왜 그럴까? 
        # ---->목표점을 0, 0으로 해뒀으니까 바보야...
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
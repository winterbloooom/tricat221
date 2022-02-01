#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import pymap3d as pm

from std_msgs.msg import UInt16, Float64
from geometry_msgs.msg import Point, Vector3
from tricat221.msg import Obstacle, ObstacleList
from visualization_msgs.msg import Marker

import gnss_converter as gc # src/gnss_converter.py
import point_class as pc # src/point_class.py


class Autonomous:
    def __init__(self):
        ## sub, pub
        rospy.Subscriber("/heading", Float64, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback)
        rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback)

        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        self.marker = rospy.Publisher("/rviz_mark", Marker, queue_size=10)

        ## 파라미터 및 변수
        self.goal_x, self.goal_y = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.goal_range = rospy.get_param("goal_range")

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

        self.psi = 0
        self.psi_desire = 0
        self.psi_goal = 0 # TODO 이름을 angle_to_goal로 할까 직관적으로?
        self.calc_angle_to_goal()

        self.obstacle = [] # 한 요소가 [begin.x, begin.y, end.x, end.y]로 되어 있음 -> TODO Point 형으로 다시 바꿀까?

        self.distance_to_goal = 0
        self.calc_distance_to_goal()

        self.error_angle = 0

        self.cnt = 0 # print 출력 속도 조절 위한 타이머

    # IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수
    def heading_callback(self, msg):
        self.psi = msg.data # [degree]
    
    # GPS로 측정한 배의 ENU 변환 좌표 콜백함수
    def boat_position_callback(self, msg):
        self.boat_x = msg.x
        self.boat_y = msg.y

    def obstacle_callback(self, msg):
        self.obsatcle = msg.obstacle
        # self.calc_angle_risk() # TODO 이 함수를 여기 위치시키는 게 맞을까? 일단 main에서 호출함

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
            begin = pc.Point()
            begin.x = ob[0] * math.cos(self.psi) - ob[1] * math.sin(self.psi)
            begin.y = ob[0] * math.sin(self.psi) + ob[1] * math.cos(self.psi)

            end = pc.Point()
            end.x = ob[2] * math.cos(self.psi) - ob[3] * math.sin(self.psi)
            end.y = ob[2] * math.sin(self.psi) + ob[3] * math.cos(self.psi)

            middle = (begin + end) / 2

            start_ang = round(begin.polar_angle_deg() - self.span_angle) # TODO 부호 합당한가 확인. lidar converter의 반시계 방향 건 참고해서
            end_ang = round(end.polar_angle_deg() + self.span_angle) # TODO round 올림 어디서 할 건지 결정
            middle_ang = round(middle.polar_angle_deg())

            # min_angle, max_angle 벗어난 것 처리함. TODO 잘 들어가는지 확인
            # TODO angle_risk 리스트를.... 딕셔너리 처리할까
            start_ang_idx = int((start_ang - self.angle_min) / self.angle_increment) if (start_ang > self.angle_min) else 0 # int 안 씌워도 되는지? # self.angle_risk.find(start_ang)
            end_ang_idx = int((end_ang - self.angle_min) / self.angle_increment) if (end_ang < self.angle_max) else len(self.angle_risk) - 1
            if middle_ang < self.angle_min:
                middle_ang_idx = 0
            elif middle_ang > self.angle_max:
                middle_ang_idx = len(self.angle_risk) - 1
            else:
                middle_ang_idx = int((middle_ang - self.angle_min) / self.angle_increment)
            
            if start_ang_idx == -1 or end_ang_idx == -1 or middle_ang_idx == -1:
                print("Error!")
            else:
                for idx in range(start_ang_idx, end_ang_idx, self.angle_increment):
                    self.angle_risk[idx] += 1*self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

            dist = middle.pc.dist_btw_points2(self.boat_x, self.boat_y) # 모듈 사용 잘 되나 확인
            dist_to_obstacles.append([dist, middle_ang_idx])
        
        dist_to_obstacles.sort() # 거리를 오름차순 정렬. 가장 가까운 장애물부터.
        for d in dist_to_obstacles:
            self.angle_risk[d[1]] += 1*self.ob_near_coefficient # TODO 잘 작동하는지 확인할 것

    def calc_goal_risk(self):
        if self.angle_min <= self.psi_goal <= self.angle_max: # 목표각 탐색 범위 안에 목표점이 존재함
            for idx in range(len(self.angle_risk)):
                ang = idx * self.angle_increment + self.angle_min
                delta_ang = abs(self.psi - ang) # TODO 값 확인하기
                if delta_ang <= self.goal_risk_range:
                    continue
                elif delta_ang <= (self.goal_risk_range * 2):
                    self.angle_risk[idx] += 1*self.goal_orient_coefficient
                elif delta_ang <= (self.goal_risk_range * 3):
                    self.angle_risk[idx] += 2*self.goal_orient_coefficient
                else:
                    self.angle_risk[idx] += 3*self.goal_orient_coefficient
        else:
            # TODO 각도 후보군의 min/max 각도 벗어난 상황은 어떻게 처리할지?
            pass

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
        print("")

        self.cnt = 0

    def view_rviz(self):
        # Arrow
        # Position/Orientation: Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height. 
        # Start/End Points: You can also specify a start/end point for the arrow, using the points member. If you put points into the points member, it will assume you want to do things this way.
        #                   The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.
        #                   scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length. 

        heading_arrow = Marker()
        heading_arrow.frame_id = "/mframe"
        heading_arrow.header.stamp = rospy.Time.now()
        heading_arrow.ns = "arrow"
        heading_arrow.action = 0 #ADD
        heading_arrow.id = 4
        heading_arrow.type = 0 #LINE_LIST
        heading_arrow.pose.orientation.y = 0 # TODO ????
        heading_arrow.pose.orientation.w = 1 # TODO ????
        heading_arrow.scale = Vector3(3,2,1)
        heading_arrow.color.r = 0.5 # purple
        heading_arrow.color.b = 0.5 # purple
        heading_arrow.color.a = 1.0 # 투명도 0
        heading = Point()
        heading.x = self.boat_x
        heading.y = self.boat_y
        heading_arrow.points.append(heading) #화살표 시작점
        heading.x = 1 * math.cos(self.psi) + self.boat_x #TODO 화살표 크기 조정할 것.
        heading.y = 1 * math.sin(self.psi) + self.boat_y
        heading_arrow.points.append(heading) # 화살표 끝점
        self.marker.publish(heading_arrow)

        psi_desire_arrow = Marker()
        psi_desire_arrow.frame_id = "/mframe"
        psi_desire_arrow.header.stamp = rospy.Time.now()
        psi_desire_arrow.ns = "arrow"
        psi_desire_arrow.action = 0 #ADD
        psi_desire_arrow.id = 5
        psi_desire_arrow.type = 0 #LINE_LIST
        psi_desire_arrow.pose.orientation.y = 0 # TODO ????
        psi_desire_arrow.pose.orientation.w = 1 # TODO ????
        psi_desire_arrow.scale = Vector3(3,2,1)
        psi_desire_arrow.color.r = 1.0 # pink
        psi_desire_arrow.color.g = 0.4 # pink
        psi_desire_arrow.color.b = 0.7 # pink
        psi_desire_arrow.color.a = 1.0 # 투명도 0
        psi_desire = Point()
        psi_desire.x = self.boat_x
        psi_desire.y = self.boat_y
        psi_desire_arrow.points.append(psi_desire) #화살표 시작점
        psi_desire.x = 1 * math.cos(self.psi_desire) + self.boat_x #TODO 화살표 크기 조정할 것.
        psi_desire.y = 1 * math.sin(self.psi_desire) + self.boat_y
        psi_desire_arrow.points.append(psi_desire) # 화살표 끝점
        self.marker.publish(psi_desire_arrow)

        boat = Marker()
        boat.header.frame_id = "/mframe"
        boat.header.stamp = rospy.Time.now()
        boat.ns = "point"
        boat.action = 0 #ADD
        boat.id = 6
        boat.type = 8 #LINE_LIST
        boat.scale.x = 0.2 #0.1
        boat.color.r = 1.0 # Yellow
        boat.color.g = 1.0 # Yellow
        boat.color.a = 1.0 # 투명도 0
        boat_position = Point()
        boat_position.x = self.boat_x
        boat_position.y = self.boat_y
        boat.points.append(boat_position)

        self.marker.publish(boat)

def main():
    rospy.init_node('Autonomous', anonymous=False)

    autonomous = Autonomous()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if autonomous.arrival_check(): # 최종 목적지에 도착함
            autonomous.servo_pub.publish(autonomous.servo_middle)
            autonomous.thruster_pub.publish() # TODO: 정지값 넣어주기
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
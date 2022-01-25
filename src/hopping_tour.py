#!/usr/bin/env python
#-*- coding:utf-8 -*-

from cgi import print_directory
import rospy
import math
import pymap3d as pm
import gnss_converter as gc # src/gnss_converter.py

from std_msgs.msg import UInt16, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu


class Hopping:
    def __init__(self):
        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/heading", Float64, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback)

        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        ## 변수 초기화
        ### waypoint 좌표(x, y) 리스트
        self.passed_waypoint = []
        self.remained_waypoint = []
        gnss_waypoint = rospy.get_param("waypoints")
        # ENU 좌표로 변환
        for waypoint in gnss_waypoint:
            enu_waypoint = gc.enu_convert(waypoint)
            self.remained_waypoint.append(enu_waypoint) # TODO extend되진 않는지 확인할 것

        self.goal_range = rospy.get_param("goal_range")

        self.error_sum_angle = 0
        self.kp_angle = rospy.get_param("kp_angle")
        self.ki_angle = rospy.get_param("ki_angle")
        self.kd_angle = rospy.get_param("kd_angle")

        self.kp_distance = rospy.get_param("kp_distance")
        self.ki_distance = rospy.get_param("ki_distance")
        self.kd_distance = rospy.get_param("kd_distance")

        self.servo_middle = rospy.get_param("servo_middle")
        self.servo_left_max = rospy.get_param("servo_left_max")
        self.servo_right_max = rospy.get_param("servo_right_max")

        self.thruster_max = rospy.get_param("thruster_max")
        self.thruster_min = rospy.get_param("thruster_min")

        ### 측정값
        self.yaw_rate = 0 # z축 각속도 [degree/s]
        self.psi = 0 # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_desire = 0

        self.boat_x = 0 # 배의 x좌표
        self.boat_y = 0 # 배의 y좌표

        self.goal_x = self.remained_waypoint[0][0] # 다음 목표의 x좌표
        self.goal_y = self.remained_waypoint[0][1] # 다음 목표의 y좌표
        
        self.distance_to_goal = 0
        self.calc_distance_to_goal() # 다음 목표까지 남은 거리
        self.error_angle = 0
        self.calc_error_angle()

        self.cnt = 0
        self.u_servo = self.servo_middle
        self.u_thruster = self.thruster_min

    # IMU z축 각속도 콜백함수
    def yaw_rate_callback(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z) # [rad/s] -> [degree/s]
    
    # IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수
    def heading_callback(self, msg):
        self.psi = msg.data # [degree]
    
    # GPS로 측정한 배의 ENU 변환 좌표 콜백함수
    def boat_position_callback(self, msg):
        self.boat_x = msg.x
        self.boat_y = msg.y
    
    def calc_distance_to_goal(self):
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)

    def distance_PID(self):
        cp_distance = self.kp_distance * self.distance_to_goal
        # ci_distance = self.ki_distance *  # dt = rate / TODO : 여기는 I 제어 필요 없을 듯?
        cd_distance = - self.kd_distance * self.distance_to_goal / 0.1 # dt = rate
        
        u_distance = cp_distance + cd_distance # + ci_distance

        u_thruster = (self.thruster_min + u_distance) 
            # m 단위인 distance 쓰러스터 제어값으로 바꾸는 법: 계수값 조정 + min/max 값 더하고 빼고
        if u_thruster > self.thruster_max:
            u_thruster = self.thruster_max

        return int(u_thruster) # TODO: 이름 다시?

    def set_next_goal(self):
        # TODO : ENU 좌표계로 잘 변환된 뒤에 작동하는지 순서 체크하기
        self.passed_waypoint.append(self.remained_waypoint[0]) # 지금 목표를 '지난 목표 리스트'에 넣음
        del self.remained_waypoint[0] # 지금 목표를 '남은 목표 리스트'에서 지움
        self.goal_x = self.remained_waypoint[0][0]
        self.goal_y = self.remained_waypoint[0][1]

    def arrival_check(self):
        self.calc_distance_to_goal() #목적지까지 거리 다시 계산
        if self.distance_to_goal <= self.goal_range:
            return True
        else:
            return False

    def calc_error_angle(self):
        # psi_desire 계산(x축(North)과 goal 사이 각)
        self.psi_desire = math.atan2(self.goal_y-self.boat_y, self.goal_x-self.boat_x) #math.atan2(y, x) -> radians, which is between PI and -PI
        self.error_angle = self.psi_desire - self.psi
    
    def error_angle_PID(self):
        cp_angle = self.kp_angle * self.error_angle # TODO : 부호 확인하기

        self.error_sum_angle += self.error_angle * 0.1 # dt = rate 
        ci_angle = self.ki_angle * self.error_sum_angle
            # 작년 코드에서 왜 이 값이 +- 90 넘지 않게 설정했었는지 알아보기 -> 임의 설정이었음
            # TODO : errorsum 초기화할 위치 선정하기

        cd_angle = - self.kd_angle * self.yaw_rate
            # 작년 코드에서 왜 de/dt를 yaw_rate로 했는지 알아보기 -> Derivative kick 현상 방지 위해 각도의 변화율인 각속도 사용
        
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
        self.calc_error_angle()
        self.u_servo = self.error_angle_PID()

        # 남은 거리 계산 -> PID로
        self.calc_distance_to_goal()
        self.u_thruster = self.distance_PID()

        self.servo_pub.publish(self.u_servo)
        self.thruster_pub.publish(self.u_thruster)

    def print_state(self):
        if self.cnt < 10:
            self.cnt += 1
            return

        print("-"*20)
        print("")   # TODO : 자릿수 맞추기
        print("Boat loc : [{0}, {1}]".format(self.boat_x, self.boat_y))
        print("Next Goal: No. {0} [{1}, {2}]".format(4-len(self.remained_waypoint)+1, self.remained_waypoint[0][0], self.remained_waypoint[0][1]))
        print("Error Angle  : {0} deg | Servo   : {1}".format(self.error_angle, self.u_servo))
        print("Dist. to Goal: {0} m | Thruster: {1}".format(self.distance_to_goal, self.u_thruster))
        print("")

        self.cnt = 0


def main():
    rospy.init_node('HoppingTour', anonymous=False)

    hopping = Hopping()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(hopping.remained_waypoint)==0:
            # 마지막 목적지까지 도착함
            hopping.servo_pub.publish(hopping.servo_middle)
            hopping.thruster_pub.publish() # TODO: 정지값 넣어주기
            print("-"*20)
            print("Finished!")
            return
        else:
            if hopping.arrival_check():
                print(hopping.boat_x, hopping.boat_y) # TODO 테스트 끝나면 지우기
                print(hopping.goal_x, hopping.goal_y) # TODO 테스트 끝나면 지우기
                print(hopping.distance_to_goal) # TODO 테스트 끝나면 지우기
                hopping.set_next_goal() # 목적지에 도착했음 -> 다음 목적지로 변경
                print("##### Arrived Current Goal. Set next goal #####")
            
            hopping.control_publish() # 계속 다음 목적지로 이동하라

        hopping.print_state()
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()

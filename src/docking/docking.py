#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
시작, ObstacleAvoidance 호출 후


    while not autonomous.is_all_connected():
        rospy.sleep(0.2)

    
    
도착했음 -> 도킹으로 전환

StartboardCam 호출, mark_detect() 수행, 천천히 전진
    True가 나오면 p1, p2 계산
        BowCam 호출
    False이면 끝 지점에 도착했는지 확인
        도착했으면 탐지 실패한 것. 후진해서 시작점으로 이동
        도착하지 않았으면 계속 함수 돌리며 전진

BowCam 호출, error_to_middle() 수행, 마크 우선 서보 모터 제어량 결정
Lidar로 장애물 계산, 스테이션 우선 서보 모터 제어량 결정
 -> 중앙에서 가장 가까운 점들의 집합을 돌출부라 가정
 -> 돌출부의 위치가 중앙을 기준으로 양 옆에 있고, 배가 지나갈 수 있는 각도에 거리 확보되어 있다면 괜찮
 -> 거리가 너무 가까우면 회피 우선
서보 모터 제어량 중 어떻게 우선순위 두어 제어해야 할 지 결정
퍼블리시
"""



############################################################################################
import rospy
import math
import pymap3d as pm

from std_msgs.msg import UInt16, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tricat221.msg import Obstacle, ObstacleList

import gnss_converter as gc # src/gnss_converter.py
import point_class as pc # src/point_class.py
import rviz_viewer as rv

import obstacle_avoidance as oa

class Docking:
    def __init__(self) -> None:
        ## subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.yaw_rate_sub = rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback, queue_size=1)

        ## publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0) # TODO 아두이노 쪽에서 S 수정하기
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=0)

        self.boat_x = 0
        self.boat_y = 0

        self.docking_start = rospy.get_param("docking_start")   # TODO :gc 적용하기
        self.docking_end = rospy.get_param("docking_end")   # TODO :gc 적용하기
        self.nxt_point_x = self.docking_start[0]
        self.nxt_point_y = self.docking_start[1]

        self.arrival_range = rospy.get_param("arrival_range")
        self.dist_to_point = 100000 #도킹 시작점 혹은 끝점까지 남은 거리

        self.psi = 0
        self.psi_desire = 0
        self.psi_goal = math.degrees(math.atan2(self.nxt_point_y, self.nxt_point_x))
        self.error_angle = 0

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
        
        self.calc_dist_to_point()

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

#     ######################## 업데이트 및 체크 관련 함수들 ########################
#     def calc_dist_to_point(self):
#         self.dist_to_point = math.hypot(self.boat_x - self.nxt_point_x, self.boat_y - self.nxt_point_y)

#     def arrival_check(self):
#         self.calc_dist_to_point() #목적지까지 거리 다시 계산
#         if self.dist_to_point <= self.arrival_range:
#             return True
#         else:
#             return False


# def main():
#     rate = rospy.Rate(10)

#     rospy.init_node('docking', anonymous=False)

#     docking = Docking()
    
#     while not docking.is_all_connected():
#         rospy.sleep(0.2)

#     print("----------All Connected, Start Obstacle Avoidance----------")

#     ob_avoidance = oa.ObstacleAvoidance()

#     while not docking.arrival_check():
#         ob_avoidance.calc_angle_risk()
#         ob_avoidance.control_publish()
#         ob_avoidance.print_state()
#         # ob_avoidance.view_rviz()
#         rate.sleep()

#     while not rospy.is_shutdown():
        

#     rospy.spin()

# if __name__ == "__main__":
#     main()



########################################################################3
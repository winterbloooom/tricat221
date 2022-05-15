#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64, Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tricat221.msg import LiDARPoint, LiDARPointList

import rviz_marker as rm
import gnss_converter as gc

"""
Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
"""

"""
# sequence ID: consecutively increasing ID 
uint32 seq

#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp

#Frame this data is associated with
string frame_id
"""

"""
float32 theta
float32 range
float32 psi_p   # goal과 점 간 에러각
"""

"""
Header header
float32[] points
"""
class LiDARConverter:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1)
        rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        
        self.result_pub = rospy.Publisher("/lidar_points", LiDARPointList, queue_size=10)
        self.rviz_lidar_pub = rospy.Publisher("/lidar_rviz", MarkerArray, queue_size=10)
        self.rviz_other_pub = rospy.Publisher("/other_rviz", MarkerArray, queue_size=10)

        
        self.raw_data = LaserScan()

        self.f_angle_increment = 5  # scan된 데이터를 이 각도 단위로 값 처리함
        self.f_angle_min = -50  # deg
        self.f_angle_max = 50   # deg

        self.f_range_max = 5    # m
        self.danger_range = 1.5 # m

        self.goal_y, self.goal_x = gc.enu_convert(rospy.get_param("autonomous_goal"))
        self.psi_goal = math.degrees(math.atan2(self.goal_y, self.goal_x))    # 한 번 넣으면 변하지 않는 값임

        self.psi = 0


    def lidar_raw_callback(self, msg):
        self.raw_data.header.seq = msg.header.seq
        self.raw_data.header.stamp = msg.header.stamp

        self.raw_data.angle_increment = msg.angle_increment
        self.raw_data.angle_min = msg.angle_min
        self.raw_data.angle_max = msg.angle_max
        self.raw_data.ranges = msg.ranges

    def heading_callback(self, msg):
        self.psi = msg.data

    def boat_position_callback(self, msg):
        self.boat_y = msg.x
        self.boat_x = msg.y

    def calc_inrange_ob(self):
        scan = self.raw_data    # LaserScan 형. 혹시나 처리 중에 self.raw_data 값이 바뀔까봐 처리함
        theta = scan.angle_min

        psi = self.psi  # heading각

        lidar_point = LiDARPoint()
        result = LiDARPointList()

        result.header = scan.header

        for range in scan.ranges:
            # TODO 지금은 라이다 기준 increment를 다 저장. 너무 많으니 내가 정한 increment로 가야 함
            # TODO 각도 지금 rad로 들어가 있음. deg로 바꿀 것
            if self.f_angle_min <= theta <= self.f_angle_max:
                if range < self.f_range_max:
                    lidar_point.range = -range    # 거리값은 (-)로 입력
                else:
                    lidar_point.range = -self.f_range_max    # max를 넘는다면 좋은 것. / 거리값은 (-)로 입력
                lidar_point.theta = theta
                lidar_point.psi_p = theta - (self.psi_goal - psi)    # (heading과 점까지 차이각) - (heading과 goal까지 차이각) = (heading과 점까지 차이각) - ((x축과 goal까지 차이각) - (x축과 heading까지 차이각))
            
                result.points.append(lidar_point)

            theta += scan.angle_increment

        return result

    def rviz_publish(self, result):
        # TODO line 말고도 다른 값들도 처리하는 순간 값으로 쓸 수 있도록 조정
        lidar_markers = MarkerArray()
        other_markers = MarkerArray()

        num_lidar_point = len(result.points)

        # lidar points
        for i, data in enumerate(result.points):
            if data.range < self.danger_range:
                color = [1, 0, 0, 1]
            else:
                color = [0, 1, 0, 1]

            p1 = [self.boat_x, self.boat_y]
            p2 = [self.boat_x + (-data.range) * math.cos(math.radians(data.theta)),\
                  self.boat_y + (-data.range) * math.sin(math.radians(data.theta))]
            line = rm.LineStrip(ns="range", id=i, color=color, scale_x=0.08, p1=p1, p2=p2)
            print(type(line.mark))
            lidar_markers.markers.append(line.mark)
        
        # heading
        heading = rm.Arrow(
            ns="heading", id=num_lidar_point+1, color=[0.5, 0.5, 0, 1],
            p1 = [self.boat_x, self.boat_y],
            p2 = [self.boat_x + 6 * math.cos(math.radians(self.psi)),
                  self.boat_y + 6 * math.cos(math.radians(self.psi))])
        other_markers.markers.append(heading)

        # goal point
        goal_point = rm.Point(
            ns="goal_point", id=num_lidar_point+1, color=[0, 0, 1, 1],
            scale=[0.2, 0.2], p=[self.goal_x, self.goal_y]
        )
        other_markers.markers.append(goal_point)

        # goal line
        goal_line = rm.LineStrip(
            ns="goal_point", id=num_lidar_point+1, color=[0, 0, 1, 1],
            scale_x=0.1,
            p1=[self.boat_x, self.boat_y],
            p2=[self.goal_x, self.goal_y]
        )
        other_markers.markers.append(goal_line)

        self.rviz_other_pub.publish(other_markers)
        self.rviz_lidar_pub.publish(lidar_markers)


def main():
    rospy.init_node('LidarConverter', anonymous=False)

    lidar_converter = LiDARConverter()
    rate = rospy.Rate(5)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        result = lidar_converter.calc_inrange_ob()
        lidar_converter.result_pub.publish(result)
        lidar_converter.rviz_publish(result)

        rate.sleep()

if __name__ == "__main__":
    main()
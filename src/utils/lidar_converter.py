#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""Subscribe 2D laser scanner(LiDAR) raw scanning data and cluster them

Notes:
    * Launch file: /tricat221/launch/sensor_test.launch의 <!-- LiDAR --> 부분 주석 해제 후 사용
    * Steps for clustering
        (a) group points
        (b) split groups
        (c) clasify Wall vs Buoy
        (d) split walls
    * Description of Paramters (/tricat221/params/lidar_params)
        * controller:
            bool 형태로, trackbar를 이용해 각 파라미터를 수정하며 실시간으로 결과를 볼 것인지 결정
        * min_input_points_size:
            LiDAR의 scanning data 개수가 일정 개수 이상이어야 계산을 시작하고, 아니라면 클러스터링 수행 및 publish 하지 않음
            scanning data가 없다면 무시한다는 의미.
        * max_gap_in_set: 
            Step (a)에서 '이 점이 근방의 점과 얼마나 떨어졌는가'를 계산하고, "max_gap_in_set" 이상이 되면 서로 다른 그룹으로 분류
            >>>>> 멀리 떨어진 점까지도 한 그룹으로 묶고 싶다면 이 값을 높게 설정
        * point_set_size:
            한 그룹의 점의 개수. 
            Step (a)에서는 '지금까지 모아온 점들의 개수'가 "point_set_size"보다 커야만 그룹들의 리스트에 추가. 그보다 적은 개수의 점만 있다면 무시.
            Step (b)에서는 '이 그룹을 둘로 분리할 때, 나눠진 두 그룹의 점의 개수'가 "point_set_size"보다 작으면 그룹 분리 안함.
            >>>>> 아주 작은 그룹도 유지하고 싶다면 이 값을 작게 설정
        * max_dist_to_ps_line:
            Step (b)에서 '이 점이 지금 속해있는 그룹으로부터 얼마나 떨어져 있는가'를 계산하고,
            "max_dist_to_ps_line" 이상이면 그 지점으로부터 두 개의 그룹으로 분리.
            >>>>> 구불구불한 그룹도 하나로 만들고 싶다면 이 값을 크게 설정
        * min_wall_length:
            Step (c)에서 '이 그룹의 길이'를 계산하고 "min_wall_length"보다 크면 부표가 아니라 벽으로 분류해 쪼개기 시작.
            >>>>> 벽을 많이 쪼개고 싶다면 이 값을 작게 설정
        * wall_particle_length:
            Step (d)에서 '그룹 시작에서 이 점까지 거리'가 "wall_particle_length" 이상이면 이 지점에서 벽을 쪼갬
            >>>>> 벽을 잘게 쪼개고 싶다면 이 값을 작게 설정
"""

import os
import sys

import cv2
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual
from datatypes.point_class import *
from datatypes.point_set_class import *
from tricat221.msg import Obstacle, ObstacleList


class Lidar_Converter:
    def __init__(self):
        # sub, pub
        rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacles", ObstacleList, queue_size=10)
        self.rviz_pub = rospy.Publisher("/rviz_visual", MarkerArray, queue_size=10)

        # params
        self.max_gap_in_set = rospy.get_param("max_gap_in_set")
        self.point_set_size = rospy.get_param("point_set_size")
        self.max_dist_to_ps_line = rospy.get_param("max_dist_to_ps_line")
        self.min_wall_length = rospy.get_param("min_wall_length")
        self.wall_particle_length = rospy.get_param("wall_particle_length")
        self.min_input_points_size = rospy.get_param("min_input_points_size")

        # on/off
        self.controller = rospy.get_param("controller")

        # scanning / converted data
        self.input_points = []  # 라이다에서 받은 모든 점들을 (x, y) 형태로 저장
        self.point_sets_list = []  # point_set들의 리스트 [ps1, ps2, ...]
        self.obstacles = []  # 최종적으로 합쳐지고 나눠인 set들 저장 (wall + buoy)

        # trackbar
        if self.controller:
            cv2.namedWindow("controller")
            cv2.createTrackbar(
                "max_gap_in_set", "controller", rospy.get_param("max_gap_in_set"), 5, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar("point_set_size", "controller", rospy.get_param("point_set_size"), 30, lambda x: x)
            cv2.createTrackbar(
                "max_dist_to_ps_line", "controller", rospy.get_param("max_dist_to_ps_line"), 5, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar(
                "min_wall_length", "controller", rospy.get_param("min_wall_length"), 50, lambda x: x
            )  # X 0.1 meter
            cv2.createTrackbar(
                "wall_particle_length", "controller", rospy.get_param("wall_particle_length"), 50, lambda x: x
            )  # X 0.1 meter

    def get_trackbar_pos(self):
        """get trackbar positions and set each values"""
        if self.controller:
            self.max_gap_in_set = cv2.getTrackbarPos("max_gap_in_set", "controller") * 0.1
            self.point_set_size = cv2.getTrackbarPos("point_set_size", "controller")
            self.max_dist_to_ps_line = cv2.getTrackbarPos("max_dist_to_ps_line", "controller") * 0.1
            self.min_wall_length = cv2.getTrackbarPos("min_wall_length", "controller") * 0.1
            self.wall_particle_length = cv2.getTrackbarPos("wall_particle_length", "controller") * 0.1

    def lidar_raw_callback(self, msg):
        """Subscribe lidar scanning data

        Notes:
            * phi
                * angle in radians
                * range: -3.14 (to left) ~ 3.14 (to right), 0 = forward
            * convert polar coordinate system -> cartesian's
            * Origin is LiDAR (boat), not ENU origin
        """
        # initialize all data lists
        self.input_points = []
        self.point_sets_list = []
        self.obstacles = []

        # save range data
        phi = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                p = Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

        # subscriber와 publisher의 sink를 맞추기 위해 이곳에서 모두 진행
        self.get_trackbar_pos()
        self.process_points()
        self.publish_obstacles()
        self.publish_rviz()

    def process_points(self):
        """Clustering process"""
        if len(self.input_points) < self.min_input_points_size:
            return

        self.group_points()
        for ps in self.point_sets_list:
            self.split_group(ps)
        self.classify_groups()

    def group_points(self):
        """Group adjacent raw scanning points

        Notes:
            "del point_set"
                파이썬은 변수 자체가 포인터 역할을 하므로,
                다른 그룹을 만들고자 단순히 이전 point_set의 인스턴스 변수들을 초기화하면
                이전에 append 되었던 point_set들도 모두 동일한 값으로 바뀜
                따라서 메모리 확보를 위해 아예 변수 지우고 다시 선언함
        """
        point_set = PointSet()
        point_set.append_point(self.input_points[0])  # 중복으로 들어가게 될 것임. 마지막에 제거할 것
        for p in self.input_points:
            if p.dist_btw_points(point_set.end) > self.max_gap_in_set:
                if point_set.set_size > self.point_set_size:
                    self.point_sets_list.append(point_set)  # set of point groups

                del point_set  # delete previous group instance
                point_set = PointSet()  # new group
                point_set.append_point(p)
            else:
                point_set.append_point(p)
        self.point_sets_list.append(point_set)  # 마지막 그룹까지 추가

        # 중복으로 들어간 첫 번째 점 제거
        self.point_sets_list[0].begin = self.point_sets_list[0].point_set[1]
        self.point_sets_list[0].set_size -= 1
        del self.point_sets_list[0].point_set[0]

    def split_group(self, ps):
        """Split group(point set) into smaller groups

        Args:
            ps (PointSet): querying point set(group)
        """
        max_distance = 0  # 그룹의 첫점 ~ 끝점 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리
        split_idx = 0  # max_distance를 가지는 점의 그룹 내 인덱스
        point_idx = 0  # 탐색하고 있는 점의 인덱스

        # find the farthest point from group
        for p in ps.point_set:
            dist_to_ps_line = ps.dist_to_point(p)  # distance from line to point
            # print(p.x, "라인까지 거리", dist_to_ps_line)
            if dist_to_ps_line > max_distance:
                max_distance = dist_to_ps_line
                split_idx = point_idx
            point_idx += 1

        # split groups
        if max_distance > self.max_dist_to_ps_line:
            # if two splitted groups would be too small, don't split this
            if split_idx < self.point_set_size or (ps.set_size - split_idx) < self.point_set_size:
                return

            ps1 = PointSet()
            ps1.input_point_set(ps.point_set[:split_idx])
            ps2 = PointSet()
            ps2.input_point_set(ps.point_set[split_idx:])

            # print("현재 ps의 인덱스", (self.point_sets_list).index(ps))
            # print("나누기 이전 psl 크기", len(self.point_sets_list))
            self.point_sets_list.insert(self.point_sets_list.index(ps), ps1)
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            self.point_sets_list.insert(self.point_sets_list.index(ps), ps2)
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            # print("3번 ps의 첫 점 x", point_sets_list[3].point_set[0].x, "ps 크기 ", len(point_sets_list[3].point_set))

            del self.point_sets_list[self.point_sets_list.index(ps)]  # 나눠서 저장한 뒤 원본 그룹 삭제

            # print("나눈 후 psl 크기 ", len(point_sets_list))
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            # print("3번 ps의 첫 점 x", point_sets_list[3].point_set[0].x, "ps 크기 ", len(point_sets_list[3].point_set))

            # split groups recursively
            self.split_group(ps1)
            self.split_group(ps2)

    def classify_groups(self):
        """Classify groups(point sets) as Walls or Buoys

        If length of group is long, classify as wall and split it into small groups.
        Append it to final clustering result list after splitting in "split_wall()" function.
        If not, just append it to final clustering list ("obstacles")
        """
        for ps in self.point_sets_list:
            if ps.dist_begin_to_end() > self.min_wall_length:
                self.split_wall(ps)
            else:
                self.obstacles.append(ps)

    def split_wall(self, ps):
        """Split long groups into short ones

        Notes:
            "del wall_particle"
                -> group_points() 함수 참고
        """
        wall_particle = PointSet()
        wall_particle.append_point(ps.begin)
        for p in ps.point_set:
            if p.dist_btw_points(wall_particle.begin) > self.wall_particle_length:
                self.obstacles.append(wall_particle)
                del wall_particle
                wall_particle = PointSet()
            wall_particle.append_point(p)
        self.obstacles.append(wall_particle)  # last group

    def publish_obstacles(self):
        """Publish clustering results in (x, y) coordinate format"""
        ob_list = ObstacleList()

        for ob in self.obstacles:
            obstacle = Obstacle()
            obstacle.begin.x = ob.begin.x
            obstacle.begin.y = ob.begin.y
            obstacle.end.x = ob.end.x
            obstacle.end.y = ob.end.y
            ob_list.obstacle.append(obstacle)

        self.obstacle_pub.publish(ob_list)

    def publish_rviz(self):
        ids = list(range(0, 100))

        input_points = []
        for p in self.input_points:
            input_points.append([p.x, p.y])
        input_points = visual.points_rviz(name="input_points", id=ids.pop(), points=input_points, color_r=255)

        filtered_points = []  # after delete too small groups after "group_points"
        for ps in self.point_sets_list:
            for p in ps.point_set:
                filtered_points.append([p.x, p.y])
        filtered_points = visual.points_rviz(
            name="filtered_points", id=ids.pop(), points=filtered_points, color_r=235, color_g=128, color_b=52
        )

        point_set = []  # after "split_group", all groups(point sets)
        for ps in self.point_sets_list:
            point_set.append([ps.begin.x, ps.begin.y])
            point_set.append([ps.end.x, ps.end.y])
        point_set = visual.linelist_rviz(
            name="point_set", id=ids.pop(), lines=point_set, color_r=55, color_g=158, color_b=54, scale=0.1
        )

        obstacle = []  # after "split_wall", final clustering results
        for ob in self.obstacles:
            obstacle.append([ob.begin.x, ob.begin.y])
            obstacle.append([ob.end.x, ob.end.y])
        obstacle = visual.linelist_rviz(
            name="obstacle", id=ids.pop(), lines=obstacle, color_r=10, color_g=81, color_b=204, scale=0.1
        )

        all_markers = visual.marker_array_rviz([input_points, filtered_points, point_set, obstacle])
        self.rviz_pub.publish(all_markers)


def main():
    rospy.init_node("LidarConverter", anonymous=False)
    lidar_converter = Lidar_Converter()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break

        rate.sleep()


if __name__ == "__main__":
    main()

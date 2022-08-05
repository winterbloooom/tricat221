#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys

import cv2
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker, MarkerArray

from datatypes.point_class import *
from datatypes.point_set_class import *
from tricat221.msg import Obstacle, ObstacleList


class Lidar_Converter:
    def __init__(self):
        # sub, pub 개체
        rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1)
        self.obstacle_pub = rospy.Publisher(
            "/obstacles", ObstacleList, queue_size=10
        )  # TODO queue_size가 publish 속도에 영향 주는지 연구!
        self.marker_array_pub = rospy.Publisher("/rviz_visual", MarkerArray, queue_size=10)

        # 파라미터 임포트
        self.max_gap_in_set = rospy.get_param("max_gap_in_set")
        self.min_point_set_size = rospy.get_param("min_point_set_size")
        self.max_dist_to_ps_line = rospy.get_param("max_dist_to_ps_line")
        self.min_wall_length = rospy.get_param("min_wall_length")
        self.min_wall_particle_length = rospy.get_param("min_wall_particle_length")
        self.min_input_points_size = rospy.get_param("min_input_points_size")
        self.controller = rospy.get_param("controller")

        # 변수
        self.lidar_header_stamp = 0
        self.lidar_header_frameid = 0
        self.input_points = []  # 라이다에서 받은 모든 점들을 (x, y) 형태로 저장
        self.point_sets_list = []  # point_set들의 리스트 [ps1, ps2, ...]
        self.obstacles = []  # 최종적으로 합쳐지고 나눠인 set들 저장 (wall + buoy)

        # trackbar
        if self.controller:
            cv2.namedWindow("controller")
            cv2.createTrackbar(
                "point ~ point in group",
                "controller",
                rospy.get_param("max_gap_in_set"),
                5,
                self.trackbar_callback,
            )  # X 0.1
            cv2.createTrackbar(
                "# of points in group",
                "controller",
                rospy.get_param("min_point_set_size"),
                30,
                self.trackbar_callback,
            )
            cv2.createTrackbar(
                "point ~ group",
                "controller",
                rospy.get_param("max_dist_to_ps_line"),
                5,
                self.trackbar_callback,
            )  # X 0.1
            cv2.createTrackbar(
                "wall should splitted",
                "controller",
                rospy.get_param("min_wall_length"),
                50,
                self.trackbar_callback,
            )  # X 0.1
            cv2.createTrackbar(
                "wall particle len",
                "controller",
                rospy.get_param("min_wall_particle_length"),
                50,
                self.trackbar_callback,
            )  # X 0.1
            cv2.createTrackbar(
                "# of input points",
                "controller",
                rospy.get_param("min_input_points_size"),
                50,
                self.trackbar_callback,
            )

    def trackbar_callback(self, usrdata):
        pass

    def get_trackbar_pos(self):
        if self.controller:
            """get trackbar poses and set each values"""
            self.max_gap_in_set = cv2.getTrackbarPos("point ~ point in group", "controller") * 0.1
            self.min_point_set_size = cv2.getTrackbarPos("# of points in group", "controller")
            self.max_dist_to_ps_line = cv2.getTrackbarPos("point ~ group", "controller") * 0.1
            self.min_wall_length = cv2.getTrackbarPos("wall should splitted", "controller") * 0.1
            self.min_wall_particle_length = (
                cv2.getTrackbarPos("wall particle len", "controller") * 0.1
            )
            self.min_input_points_size = cv2.getTrackbarPos("# of input points", "controller")

    def lidar_raw_callback(self, msg):
        self.input_points = []
        self.point_sets_list = []
        self.obstacles = []
        self.lidar_header_stamp = msg.header.stamp
        self.lidar_header_frameid = msg.header.frame_id
        # print(self.lidar_header_stamp)
        # if (
        #     int(str(self.lidar_header_stamp)) > 1651207343593234463
        # ):  # 1659356681634095032: #1659356677774594514: #
        #     return

        phi = msg.angle_min  # 각 점의 각도 계산 위해 계속 누적해갈 각도
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                p = Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

        self.get_trackbar_pos()
        self.process_points()

    def process_points(self):
        if len(self.input_points) < self.min_input_points_size:
            return  # TODO 다 가려졌을 때는 아예 이전 상태로 유지라서, 아예 점 없어지는 처리 필요

        self.group_points()

        for ps in self.point_sets_list:
            self.split_group(ps)

        self.classify_groups()

        self.publish_obstacles()

        self.publish_rviz()

    def group_points(self):
        point_set = Point_Set()
        point_set.append_point(self.input_points[0])  # 중복으로 들어가게 될 것임. 마지막에 제거 해줌
        for p in self.input_points:
            if p.dist_btw_points(point_set.end) > self.max_gap_in_set:
                if point_set.set_size > self.min_point_set_size:
                    self.point_sets_list.append(point_set)
                    # TODO 크기 따라 작은지 큰지 bool 변경해주는 부분 추가 -> 그냥 아예 리스트 추가 자체를 안 하는 걸로

                del point_set
                # 파이썬은 변수들 자체가 포인터 역할 하므로
                # 단순히 point_set의 인스턴스 변수들을 초기화하면
                # append 되는 point_set들이 다 동일한 것이 들어감(앞의 것도 바뀜)
                # 그래서 메모리 확보 위해 아예 변수 지우고 다시 선언함

                point_set = Point_Set()
                point_set.append_point(p)
            else:
                point_set.append_point(p)
        self.point_sets_list.append(point_set)  # 마지막 set 리스트에 빼먹지 않고 추가해줌

        ##### 중복으로 들어간 첫 번째 점 제거함 / TODO IndexError: list index out of range
        # print(self.point_sets_list[0].begin)
        # print(self.point_sets_list[0])
        # print(self.point_sets_list[0].set_size)
        # print(self.point_sets_list[0].point_set[1])
        self.point_sets_list[0].begin = self.point_sets_list[0].point_set[1]
        self.point_sets_list[0].set_size -= 1
        # TODO min_point_set_size 처리...?
        del self.point_sets_list[0].point_set[0]

    def split_group(self, ps):
        max_distance = 0  # point_set의 첫점부터 끝점까지 이은 직선으로부터 가장 멀리 떨어진 점까지의 거리
        split_idx = 0  # TODO : 잘 동작하는지 확인하기
        point_idx = 0

        for p in ps.point_set:
            dist_to_ps_line = ps.dist_to_point(p)  # print(p.x, "라인까지 거리", dist_to_ps_line)
            if dist_to_ps_line > max_distance:
                max_distance = dist_to_ps_line
                split_idx = point_idx
            point_idx += 1

        if max_distance > self.max_dist_to_ps_line:
            if (
                split_idx < self.min_point_set_size
                or (ps.set_size - split_idx) < self.min_point_set_size
            ):
                return

            ps1 = Point_Set()
            ps1.input_point_set(ps.point_set[:split_idx])
            ps2 = Point_Set()
            ps2.input_point_set(
                ps.point_set[split_idx:]
            )  # ps2 = Point_Set(ps.point_set[split_idx:])

            # print("현재 ps의 인덱스", (point_sets_list).index(ps))
            # print("나누기 이전 psl 크기", len(point_sets_list))
            self.point_sets_list.insert(self.point_sets_list.index(ps), ps1)
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            self.point_sets_list.insert(
                self.point_sets_list.index(ps), ps2
            )  # TODO 이거 전후 위치도 확인하기. 분할된 것도 순서 지켜서 들어가는지
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            # print("3번 ps의 첫 점 x", point_sets_list[3].point_set[0].x, "ps 크기 ", len(point_sets_list[3].point_set))

            del self.point_sets_list[self.point_sets_list.index(ps)]

            # print("나눈 후 psl 크기 ", len(point_sets_list))
            # print("0번 ps의 첫 점 x", point_sets_list[0].point_set[0].x, "ps 크기 ", len(point_sets_list[0].point_set))
            # print("1번 ps의 첫 점 x", point_sets_list[1].point_set[0].x, "ps 크기 ", len(point_sets_list[1].point_set))
            # print("2번 ps의 첫 점 x", point_sets_list[2].point_set[0].x, "ps 크기 ", len(point_sets_list[2].point_set))
            # print("3번 ps의 첫 점 x", point_sets_list[3].point_set[0].x, "ps 크기 ", len(point_sets_list[3].point_set))
            self.split_group(ps1)
            self.split_group(ps2)

    def classify_groups(self):
        for ps in self.point_sets_list:
            if ps.dist_begin_to_end() > self.min_wall_length:  # 길이 길어서 벽으로 분류
                self.split_wall(ps)  # 어느 정도 길이로 벽을 쪼개서 obstacle에 넣음
            else:
                self.obstacles.append(ps)  # 부표로 인식하고 바로 obstacle에 넣어줌
            # self.obstacles.append(ps)

    def split_wall(self, ps):
        wall_particle = Point_Set()
        wall_particle.append_point(ps.begin)

        for p in ps.point_set:
            if p.dist_btw_points(wall_particle.begin) > self.min_wall_particle_length:
                self.obstacles.append(wall_particle)

                del wall_particle
                wall_particle = Point_Set()
                # wall_particle.append_point(p)
            # else:
            # wall_particle.append_point(p)
            wall_particle.append_point(p)
        self.obstacles.append(wall_particle)  # 마지막 파티클까지 잊지 않고 넣어줌

    def publish_obstacles(self):
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
        marker_array = MarkerArray()

        input_points = Marker()  # input_points가 잘 받아지고 변환 잘 되었는지 확인용
        input_points.header.frame_id = "/laser"
        input_points.header.stamp = rospy.Time.now()
        input_points.ns = "inputPoints"
        input_points.action = 0  # ADD
        input_points.pose.orientation.w = 0.0  # ???
        input_points.id = 0
        input_points.type = 8  # POINTS
        input_points.scale.x = 0.1
        input_points.scale.y = 0.1
        input_points.color.r = 1.0  # Red
        input_points.color.a = 1.0  # 투명도 0
        for p in self.input_points:
            point = Point32()
            point.x = p.x
            point.y = p.y
            input_points.points.append(point)

        filtered_points = Marker()  # input_points 중 따로 노는 점을 제거한 후 확인용
        filtered_points.header.frame_id = "/laser"
        filtered_points.header.stamp = rospy.Time.now()
        filtered_points.ns = "filteredPoints"
        filtered_points.action = 0  # ADD
        filtered_points.pose.orientation.w = 0.0  # ???
        filtered_points.id = 1
        filtered_points.type = 8  # POINTS
        filtered_points.scale.x = 0.1
        filtered_points.scale.y = 0.1
        filtered_points.color.r, filtered_points.color.g, filtered_points.color.b = (
            235 / 255.0,
            128 / 255.0,
            52 / 255.0,
        )
        filtered_points.color.a = 1.0  # 투명도 0
        for ps in self.point_sets_list:
            for p in ps.point_set:
                point = Point32()
                point.x = p.x
                point.y = p.y
                filtered_points.points.append(point)

        point_set = Marker()  # 각 point_set을 확인용
        point_set.header.frame_id = "/laser"
        point_set.header.stamp = rospy.Time.now()
        point_set.ns = "pointSet"
        point_set.action = 0  # ADD
        point_set.pose.orientation.w = 1.0  # ???
        point_set.id = 2
        point_set.type = 5  # LINE_LIST
        point_set.scale.x = 0.1
        point_set.color.r, point_set.color.g, point_set.color.b = (
            55 / 255.0,
            158 / 255.0,
            54 / 255.0,
        )
        point_set.color.a = 1.0  # 투명도 0
        for ps in self.point_sets_list:
            point = Point32()
            point.x = ps.begin.x
            point.y = ps.begin.y
            point_set.points.append(point)
            point = Point32()
            point.x = ps.end.x
            point.y = ps.end.y
            point_set.points.append(point)

        obstacle = Marker()  # 벽까지 다 구분된 파티클 확인용
        obstacle.header.frame_id = "/laser"
        obstacle.header.stamp = rospy.Time.now()
        obstacle.ns = "obstacle"
        obstacle.action = 0  # ADD
        obstacle.pose.orientation.w = 1.0  # ???
        obstacle.id = 3
        obstacle.type = 5  # LINE_LIST
        obstacle.scale.x = 0.1
        obstacle.color.r, obstacle.color.g, obstacle.color.b = 10 / 255.0, 81 / 255.0, 204 / 255.0
        obstacle.color.a = 1.0  # 투명도 0
        for ob in self.obstacles:
            point = Point32()
            point.x = ob.begin.x
            point.y = ob.begin.y
            obstacle.points.append(point)
            point = Point32()
            point.x = ob.end.x
            point.y = ob.end.y
            obstacle.points.append(point)

        marker_array.markers.append(input_points)
        marker_array.markers.append(filtered_points)
        marker_array.markers.append(point_set)
        marker_array.markers.append(obstacle)

        self.marker_array_pub.publish(marker_array)


def main():
    rospy.init_node("LidarConverter", anonymous=False)

    lidar_converter = Lidar_Converter()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()


######## TRASH ################
# class Lidar_Converter:
# def __init__(self):
#     rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback)

#         self.max_point_set_gap = rospy.get_param("max_point_set_gap")
#         self.min_point_set_size = rospy.get_param("min_point_set_size")
#         self.max_point_set_range = rospy.get_param("max_point_set_range")

#         self.input_points = []
#         self.point_sets = []

#     # def polar_to_cartesian(self, r, phi):
#     #     # r: 반지름 / phi: 각도
#     #     # 극좌표계를 직교좌표계로 변환함
#     #     return [r * math.cos(phi), r * math.sin(phi)]  # TODO : 문법 맞나 확인

#     def lidar_raw_callback(self, msg):
#         # base_frame_id_ = scan_msg->header.frame_id;
#         # stamp_ = scan_msg->header.stamp;
#         idx = 0
#         phi = msg.angle_min # 각 점의 각도 계산 위해 계속 누적해갈 각도
#         for r in msg.ranges:
#             if msg.range_min <= r <= msg.range_max:
#                 p = Point(idx, r, phi)
#                 self.input_points.append(p)
#                 # self.input_points.append(self.polar_to_cartesian(r, phi)) # TODO : 작동하나 확인
#             idx += 1
#             phi += msg.angle_increment

#         self.process_points()

#     def process_points(self):
#         self.group_points()
#         # self.filter_seperated_point()
#         self.split_group()

#     def group_points(self):
#         temp_point_set = []
#         # point_set_size = 0
#         for p in self.input_points:
#             if len(temp_point_set)==0: #point_set_size==0
#                 temp_point_set.append(p)

#             if p.distance_between_points(temp_point_set[-1]) > self.max_point_set_gap: # 마지막 점부터 지금 탐색 중인 점까지 거리가 멀어서 하나의 set으로 볼 수 없음
#                 if len(temp_point_set) >= self.min_point_set_size: # if point_set_size >= self.min_point_set_size # 지금까지 모아온 set의 크기(점의 개수)가 충분히 큼(적지 않음)
#                     self.point_sets.append(temp_point_set) # point_set으로 인정하고 추가함. 그렇지 않으면 그냥 무시하고 set으로 넣지 않음
#                 temp_point_set = [] # 초기화
#                 # point_set_size = 0
#             else:
#                 temp_point_set.append(p)

#         if len(temp_point_set) >= self.min_point_set_size: # 마지막 temp_point_set도 처리하기 위해 한 번 더 추가함
#             self.point_sets.append(temp_point_set)

#     # def filter_seperated_point(self):
#     #     for ps in self.point_sets:
#     #         if len(ps) < self.min_point_set_size:
#     #             del

#     def split_group(self):
#         for ps in self.point_sets:
#             start_point_idx = 0
#             end_point_idx = 1
#             for i in range(2, len(ps)): # 첫 번째, 두 번째 점은 그냥 넘어감 -> TODO: 정말 넘어가도 되는지 확인 필요?
#                 dist_to_line = self.calc_dist_to_line(ps[i], start_point_idx, end_point_idx)
#                 if dist_to_line > self.max_point_set_range:


#     # 직선과 점 사이 거리
#     def calc_dist_to_line(self, point, start_point, end_point):
#         vec_a = np.array([end_point.x - start_point.x, end_point.y - start_point.y])
#         vec_b = np.array([point.x - start_point.x, point.y - start_point.y])
#         vec_ortho_projection = vec_a.dot(vec_b)
#         foot_of_perpendicular = vec_ortho_projection * vec_a / (vec_a[0]**2 + vec_a[1]**2)
#         point_to_fop = np.array([point.x, point.y]) - foot_of_perpendicular
#         return math.sqrt(point_to_fop[0]**2 + point_to_fop[1]**2)
#         # double distanceTo(const Point& p) return (p - projection(p)).length()
#         # Point projection(const Point& p) const {
#             # Point a = last_point - first_point;
#             # Point b = p - first_point;
#             # return first_point + a.dot(b) * a / a.lengthSquared();
#             # }

#     def projection(self, point):

#     def classify_group(self):
#         pass

#     def calc_desire_angle(self):
#         pass

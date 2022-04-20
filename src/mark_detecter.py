#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy

class StarboardCam:
    def __init__(self) -> None:
        self.p1 = 0 # 좌측 끝단부터 표지 중점까지 길이(픽셀 x좌표 차)
        self.p2 = 0 # 표지 중점부터 중앙 수직선까지 길이(픽셀 x좌표 차)

        self.target_mark = rospy.get_param("target_mark")

    def mark_detect(self):
        """
        카메라를 열어 프레임 단위로 표지 존재 여부 및 목표 표지 검출
        검출되면 T/F 리턴 -> 검출은 됐지만 원하는 비율이 아님 / 검출도 됐고 원하는 비율임 / 검출 안 됐음 으로 나눠 리턴?
        """
        pass

    # def mark_position(self):
    #     """
    #     p1, p2 리턴
    #     """
    #     return self.p1, self.p2


class BowCam:
    def __init__(self) -> None:
        self.frame_mid = 0
        self.mark_mid = 0

    def mark_check(self):
        """
        원하는 마크가 맞는지 체크. 확인 후에는 detection이 아닌 tracking을 중심으로 사용
        """
        pass

    def error_to_middle(self):
        """
        표지 중앙과 카메라 중앙이 맞는지 확인 후 에러값 리턴
        """
        return (self.frame_mid - self.mark_mid)
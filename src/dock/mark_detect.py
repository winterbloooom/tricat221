#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""search marks in an input image and detect target mark

mark_detector.py
===========

Todo
    * 코드 정리하기
    * 테스트 파일 모드를 ros 실시간 모드 혹은 이미지 모드 두 개로 선택해 구현
"""


import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def mean_brightness(img):
    """평균 밝기로 화면 밝기 조정"""
    fixed = 100  # 이 값 주변으로 평균 밝기 조절함
    m = cv2.mean(img)  # 평균 밝기
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv2.add(img, scalar)

    return dst


def preprocess_image(raw_img, hsv=True, blur=False, brightness=False):
    """preprocess the raw input image

    brightness / gaussian blur / color space convert

    Args:
        raw_img (np.ndarray): Input raw image from the camera
        hsv (bool): Whether to convert color space into HSV
        blur (bool): Whether to do Gaussian Blur or not. Fix kernel size with 5
        brightness (bool): Whether to adjust brightness with mean brightness value

    Returns:
        np.ndarray: preprocessed image
    """
    img = raw_img

    if brightness == True:
        img = mean_brightness(img)
    if blur == True:
        img = cv2.GaussianBlur(img, (5, 5), 0)
    if hsv == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    return img


def select_color(img, range, color_space="hsv"):
    """filter color according to color range settings to pick specific target color

    Args:
        img (np.ndarray): preprocessed image with 3 channels
        range (list): (3 X 2) list that limits color space range. For row, [lower bound, upper bound]. Columns are color space elements.
        color_space (str): Select color space to convert input image. hsv(default), rgb

    Returns:
        np.ndarray: gray-scale image with specific color range

    Todo:
        * 색공간 따라 변경하기
    """
    selceted = cv2.inRange(img, range[0], range[1])
    return selceted


def detect_target(img, target_shape, mark_detect_area, draw_contour=True):
    """detect all marks and get target information if target mark is there

    1. 모폴로지 연산으로 빈 공간 완화 & 노이즈 제거
    2.

    Args:
        img (np.ndarray): image only with target color
        target_shape (int): number of sides of target shape. 3, 4, or over
        draw_contour (bool): Whether to draw detected marks

    Returns:
        list : target detected information. [area of target (in pixel), location of target in the image (in pixel, only column)]

    Todo:
        * 몇 개의 도형이 검출되었고, 몇 개가 목표에 해당하는지, 몇 개가 각각 어디에 해당하는지 나타내기

    Note:
        * approxPolyDP()의 반환인 approx의 구조 (삼각형일 때)
            * type: numpy.ndarray
            * shape: (3, 1, 2) -> 변 개수, 1, [행, 열] 정보이므로 요소 두 개
            * 예: [[[105, 124]], [[107, 205]], [[226, 163]]]
        * 때에 따라서는 같은 도형이 여러 개 발견될 수도 있음.
            * 다른 모양의 도형이 프레임에서 잘린다든지, 다른 물체를 오인한다든지
            * 일단 너비가 최대인 것을 따라가도록 설정
    """
    detected = False
    targets = []  # [area, center_col]

    morph_kernel = np.ones((9, 9), np.uint8)
    morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel)

    shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)  # 시각화할 image

    _, contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)  # 발견된 도형들

    for contour in contours:
        approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)  # 도형 근사
        area = cv2.contourArea(approx)  # 도형 넓이
        if area < mark_detect_area:
            continue  # 너무 작은 것은 제외

        # print("-" * 30)
        # print("area : {} > {}".format(area, mark_detect_area))

        vertex_num = len(approx)  # 변의 개수
        # print("# of vertices : {}".format(vertex_num))

        if vertex_num == 3:  # 삼각형이 탐지됨
            # print("Shape : Triangle")
            shape, _, center_col = draw_mark(
                shape, target_shape, approx, vertex_num, area
            )  # 도형 그리기
            if target_shape == 3:  # 타겟이 삼각형이라면
                detected = True
                targets.append([area, center_col])
        elif vertex_num == 4:  # 사각형이 탐지됨
            # print("Shape : Rectangle")
            shape, _, center_col = draw_mark(shape, target_shape, approx, vertex_num, area)
            if target_shape == 4:
                detected = True
                targets.append([area, center_col])
        else:  # 원이 탐지됨
            _, radius = cv2.minEnclosingCircle(approx)  # 원으로 근사
            ratio = radius * radius * 3.14 / area  # 해당 넓이와 정원 간의 넓이 비
            if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                # print("Shape : Circle")
                shape, _, center_col = draw_mark(shape, target_shape, approx, vertex_num, area)
                if target_shape == 5:
                    detected = True
                    targets.append([area, center_col])

    if draw_contour:
        shape = cv2.line(shape, (320, 0), (320, 480), (255, 0, 0), 2)

    if detected:
        if len(targets) == 1:  # 타겟 마크가 하나만 검출됨
            return targets[-1], shape
        else:  # 타겟이 여러 개 검출됨. 0번째 요소인 넓이가 가장 큰 마크를 타겟으로 함
            max_area_idx = targets.index(max(targets))
            return targets[max_area_idx], shape
    else:  # 타겟이 검출되지 않음
        return [], shape


def draw_mark(window, target_shape, contour, vertices, area):
    """visualize the results

    Args:
        window (numpy.ndarray) : image to draw things (color selected image)
        target_shape (int) : target shape(number of vertices). 3(triangle), 4(rectangle), 5(circle)
        contour (list) : approximated contour to draw
        vertices (int) : number of vertices of this contour

    Returns:
        tuple : (numpy.ndarray, int, int) = (image after drawing, center row coord (pixel), center col coord (pixel))

    """
    box_left_top = (min(contour[:, 0, 0]), min(contour[:, 0, 1]))  # 도형에 박스를 그렸을 때의 좌상단
    box_right_bottom = (max(contour[:, 0, 0]), max(contour[:, 0, 1]))  # 도형에 박스를 그렸을 때의 우하단

    center_col = int((box_left_top[0] + box_right_bottom[0]) / 2)
    # 도형 중앙 세로 위치(row). sum(contour[:, 0, 0]) / vertices로 계산할 수도 있음
    center_row = int((box_left_top[1] + box_right_bottom[1]) / 2)
    # 도형 중앙 가로 위치(col). sum(contour[:, 0, 1]) / vertices로 계산할 수도 있음

    if target_shape == len(contour) or (vertices >= 5 and target_shape == 5):
        color = (0, 255, 0)  # target은 초록색
        window = cv2.line(window, (center_col, 0), (center_col, 480), (0, 0, 255), 2)
        # TODO 검출된 모든 정답값에 그림 -> 최댓값의 크기만 그리고 싶은데..
    else:
        color = (135, 219, 250)  # 그 외는 노란색

    if vertices == 3:
        shape = "Triangle"
    elif vertices == 4:
        shape = "Rectangle"
    else:
        shape = "Circle"
    # caption = "{} ({}, {})".format(shape, center_row, center_col)  # 도형에 보일 텍스트(중점 좌표)
    caption = "{} ( {:,d} )".format(shape, int(area))  # 도형에 보일 텍스트(넓이)

    window = cv2.drawContours(window, [contour], -1, color, -1)  # 도형 그리기
    window = cv2.rectangle(window, box_left_top, box_right_bottom, color, 1)  # 박스 그리기
    window = cv2.circle(window, (center_col, center_row), 2, (0, 0, 255), 2)  # 중심점 그리기
    window = cv2.putText(
        window, caption, box_left_top, cv2.FONT_HERSHEY_PLAIN, 1, color, 1, cv2.LINE_AA
    )  # 글씨 쓰기

    return window, center_row, center_col  # 잘 안그려지면 window 새로 할당하기


def get_trackbar_pos(color_range):
    """for test_with_img()"""
    color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
    color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
    color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
    color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
    color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
    color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
    mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller") * 100
    return color_range, mark_detect_area


def test_with_img():
    # variables
    target_shape = 4
    color_range = np.array([[37, 104, 123], [100, 158, 193]])
    area = 0
    mark_detect_area = 75
    cv2.namedWindow("controller")

    # image load
    path_prefix = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
    raw_img = cv2.imread(path_prefix + "/pic1.jpeg", cv2.IMREAD_COLOR)
    raw_img = cv2.rotate(raw_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raw_img = cv2.resize(raw_img, (640, 480))  # (504, 672))
    if raw_img is None:
        print("Image load failed!")
        exit(1)

    # trackbar load
    cv2.namedWindow("controller")
    cv2.createTrackbar("color1 min", "controller", color_range[0][0], 180, lambda x: x)
    cv2.createTrackbar("color1 max", "controller", color_range[1][0], 180, lambda x: x)
    cv2.createTrackbar("color2 min", "controller", color_range[0][1], 255, lambda x: x)
    cv2.createTrackbar("color2 max", "controller", color_range[1][1], 255, lambda x: x)
    cv2.createTrackbar("color3 min", "controller", color_range[0][2], 255, lambda x: x)
    cv2.createTrackbar("color3 max", "controller", color_range[1][2], 255, lambda x: x)
    cv2.createTrackbar(
        "mark_detect_area", "controller", mark_detect_area, 100, lambda x: x
    )  # X 100 하기

    while True:
        cv2.moveWindow("controller", 0, 0)
        color_range, mark_detect_area = get_trackbar_pos(color_range)
        preprocessed = preprocess_image(raw_img, blur=True)
        hsv_img = select_color(preprocessed, color_range)
        target, shape_img = detect_target(hsv_img, target_shape, mark_detect_area)

        show_img = np.hstack([raw_img, shape_img])
        cv2.imshow("controller", show_img)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


# test_with_img()

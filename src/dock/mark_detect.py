#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""search marks in an input image and detect target mark"""


import os
import sys

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


def mean_brightness(img):
    """평균 밝기로 화면 밝기 조정

    Args:
        img (numpy.ndarray): 입력 이미지
    Returns:
        dst (numpy.ndarray): 평균 밝기가 조절된 이미지
    """
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


def select_color(img, range):
    """filter color according to color range settings to pick specific target color

    Args:
        img (np.ndarray): preprocessed image with 3 channels
        range (list): (3 X 2) list that limits color space range. For row, [lower bound, upper bound]. Columns are color space elements.

    Returns:
        np.ndarray: gray-scale image with specific color range
    """
    selceted = cv2.inRange(img, range[0], range[1])
    return selceted


def is_target(target_shape, target_detect_area, vertex_num, area):
    """지금 보고 있는 표식이 타겟인지 판단

    Args:
        target_shape (int): 타겟의 모양
        target_detect_area (float): 이 이상의 넓이 가져야 타겟임
        vertex_num (int): 탐지된 모양
        area (float): 탐지된 넓이

    Retuns:
        bool: True(타겟임) / False(타겟이 아니거나 기준에 못 미침)
    """
    if (vertex_num == target_shape) and (area >= target_detect_area):
        # 모서리 개수가 일치하고, 넓이가 충분히 크면
        return True
    else:
        return False


def detect_target(img, target_shape, mark_detect_area, target_detect_area, draw_contour=True):
    """detect all marks and get target information if target mark is there

    1. 모폴로지 연산으로 빈 공간 완화 & 노이즈 제거
    2. 도형 검출 및 근사
    3. 넓이 작은 것은 제외
    4. 변의 개수로 타겟 판단 및 시각화

    Args:
        img (np.ndarray): image only with target color
        target_shape (int): number of sides of target shape. 3, 4, or over
        draw_contour (bool): Whether to draw detected marks

    Returns:
        target (list): target detected information. [area of target (in pixel), location of target in the image (in pixel, only column)]
        shape (numpy.ndarray): detection results
        max_area (float): max area (if target is not detected, 0)

    Note:
        * approxPolyDP()의 반환인 approx의 구조 (삼각형일 때)
            * type: numpy.ndarray
            * shape: (3, 1, 2) -> 변 개수, 1, [행, 열] 정보이므로 요소 두 개
            * 예: [[[105, 124]], [[107, 205]], [[226, 163]]]
        * 때에 따라서는 같은 도형이 여러 개 발견될 수도 있음.
            * 다른 모양의 도형이 프레임에서 잘린다든지, 다른 물체를 오인한다든지
            * 일단 너비가 최대인 것을 따라가도록 설정
    """
    # 기본 변수 선언
    detected = False  # 타겟을 발견했는가
    max_area = 0  # 가장 넓은 넓이의 도형
    target = []  # 타겟 정보 [area, center_col, approx]

    # 모폴로지 연산
    morph_kernel = np.ones((9, 9), np.uint8)
    morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel)

    # 시각화 결과 영상 생성
    shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)  # 시각화할 image
    shape = cv2.line(shape, (320, 0), (320, 480), (255, 0, 0), 2)  # 중앙 세로선

    # 화면 내 모든 도형 검출
    _, contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    # 각 도형의 타겟 여부 탐지
    for contour in contours:
        # 도형 근사
        approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)

        # 도형 넓이
        area = cv2.contourArea(approx)
        if area < mark_detect_area:
            continue  # 너무 작은 것은 제외
        # print("area : {} / {} / {}".format(area, mark_detect_area, target_detect_area))

        # 변의 개수
        vertex_num = len(approx)
        # print("# of vertices : {}".format(vertex_num))

        # 탐지 도형 구분
        # 삼각형
        if vertex_num == 3:
            detected = is_target(target_shape, target_detect_area, vertex_num=3, area=area)
        # 십자가
        elif vertex_num == 12:
            detected = is_target(target_shape, target_detect_area, vertex_num=12, area=area)
        # 원
        else:
            _, radius = cv2.minEnclosingCircle(approx)  # 원으로 근사
            ratio = radius * radius * 3.14 / (area + 0.000001)  # 해당 넓이와 정원 간의 넓이 비
            if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                detected = is_target(target_shape, target_detect_area, vertex_num=5, area=area)

        # 탐지된 도형의 중앙 지점, 도형을 감싸는 사각형 꼭짓점 좌표
        box_points, center_point = contour_points(approx)

        # 타겟 정보 저장 (해당 화면에서 타겟이 검출되었다면)
        if detected:
            if area > max_area:  # 최대 크기라면 정보 갱신
                target = [area, center_point[1]]
                max_area = area
            else:
                detected = False  # 더 큰 마크가 있으므로 무시

        # 현재 화면 검출 결과 시각화
        shape = draw_mark(
            window=shape,
            contour=approx,
            vertices=vertex_num,
            area=area,
            box_points=box_points,
            center_point=center_point,
            is_target=detected,
        )

    # 타겟 검출 결과 반환
    if max_area != 0:
        return target, shape, max_area
    else:  # 타겟이 검출되지 않음
        return [], shape, max_area


def contour_points(contour):
    """도형을 감싸는 사각형과 도형의 중앙 지점 정보 추출"""
    box_left_top = (min(contour[:, 0, 0]), min(contour[:, 0, 1]))  # 도형에 박스를 그렸을 때의 좌상단
    box_right_bottom = (max(contour[:, 0, 0]), max(contour[:, 0, 1]))  # 도형에 박스를 그렸을 때의 우하단

    center_col = int((box_left_top[0] + box_right_bottom[0]) / 2)
    # 도형 중앙 세로 위치(row). sum(contour[:, 0, 0]) / vertices로 계산할 수도 있음
    center_row = int((box_left_top[1] + box_right_bottom[1]) / 2)
    # 도형 중앙 가로 위치(col). sum(contour[:, 0, 1]) / vertices로 계산할 수도 있음

    return [box_left_top, box_right_bottom], [center_row, center_col]


def draw_mark(window, contour, vertices, area, box_points, center_point, is_target=False):
    """visualize the results

    Args:
        window (numpy.ndarray) : image to draw things (color selected image)
        contour (list) : approximated contour to draw
        vertices (int) : number of vertices of this contour
        area (float): area of this contour
        box_points (list): left-top, right-bottom coordinate of box which includes this contour
        center_point (list): center of this contour
        is_target (bool): whether this contour is a target

    Returns:
        window (numpy.ndarray): draw this contour and its information
    """
    # 도형
    if is_target:
        color = (0, 255, 0)  # target은 초록색으로 표시
        window = cv2.line(window, (center_point[1], 0), (center_point[1], 480), (0, 0, 255), 2)
    else:
        color = (135, 219, 250)  # 그 외는 노란색

    # 도형 정보 라벨
    if vertices == 3:
        shape = "Triangle"
    elif vertices == 12:
        shape = "Cross"
    else:
        shape = "Circle"
    caption = "{} ( {:,d} )".format(shape, int(area))  # 도형에 보일 텍스트(도형 종류, 넓이)

    window = cv2.drawContours(window, [contour], -1, color, -1)  # 도형 그리기
    window = cv2.rectangle(window, box_points[0], box_points[1], color, 1)  # 박스 그리기
    window = cv2.circle(window, (center_point[1], center_point[0]), 2, (0, 0, 255), 2)  # 중심점 그리기
    window = cv2.putText(window, caption, box_points[0], cv2.FONT_HERSHEY_PLAIN, 1, color, 1, cv2.LINE_AA)  # 글씨 쓰기

    return window


def get_trackbar_pos(color_range):
    """test_with_img() 함수에서 사용할 트랙바"""
    color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
    color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
    color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
    color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
    color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
    color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
    mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller") * 100
    return color_range, mark_detect_area


def test_with_img():
    """사진 하나로 기능 테스트"""
    # variables
    target_shape = 12
    color_range = np.array([[81, 124, 0], [128, 182, 255]])
    area = 0
    mark_detect_area = 75
    cv2.namedWindow("controller")

    # image load
    path_prefix = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
    raw_img = cv2.imread(path_prefix + "/0816-2.png", cv2.IMREAD_COLOR)
    # raw_img = cv2.rotate(raw_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raw_img = raw_img[:240, :320]  # (504, 672))
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
    cv2.createTrackbar("mark_detect_area", "controller", mark_detect_area, 100, lambda x: x)  # X 100 하기

    while True:
        cv2.moveWindow("controller", 0, 0)
        color_range, mark_detect_area = get_trackbar_pos(color_range)
        preprocessed = preprocess_image(raw_img, blur=True)
        hsv_img = select_color(preprocessed, color_range)
        target, shape_img, _ = detect_target(hsv_img, target_shape, mark_detect_area, 100000)

        # raw_img = cv2.resize(raw_img, (320, 240)) #dsize=(0, 0), fx=0.5, fy=0.5)
        # hsv_img = cv2.resize(hsv_img, (320, 240)) #dsize=(0, 0), fx=0.5, fy=0.5)
        hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_GRAY2BGR)
        col1 = np.vstack([raw_img, hsv_img])
        cv2.imshow("col1", raw_img)
        shape_img = cv2.resize(hsv_img, (640, 480))
        col2 = cv2.resize(shape_img, dsize=(0, 0), fx=0.9, fy=1.0)
        print("")
        print(col1.shape)
        print(col2.shape)
        show_img = np.hstack([col1, col2])

        cv2.imshow("controller", show_img)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


# test_with_img()

#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""search marks in an input image and detect target mark

mark_detector.py
===========

Todo
    * 코드 정리하기
"""

import numpy as np
import cv2

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

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

    if brightness==True:
        pass
    if blur==True:
        img = cv2.GaussianBlur(img, (5, 5), 0)
    if hsv==True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    return img


def select_color(img, range, color_space="hsv"):
    """
    Args:
        img (np.ndarray): preprocessed image with 3 channels
        range (list): (3 X 2) list that limits color space range. For row, [lower bound, upper bound]. Columns are color space elements.
        color (str): Select color space to convert input image. hsv(default), rgb
    
    Returns:
        np.ndarray: gray-scale image with specific color range

    Todo:
        * 0 또는 1로만 바꾸기
        * 색공간 따라 변경하기
        * 더 다듬기
    """

    # channels = cv2.split(img)
    # channels[0] = cv2.inRange(channels[0], range[0][0], range[0][1])
    # channels[1] = cv2.inRange(channels[1], range[1][0], range[1][1])
    # channels[2] = cv2.inRange(channels[2], range[2][0], range[2][1])

    # cv2.imshow("0", channels[0])
    # cv2.imshow("1", channels[1])
    # cv2.imshow("2", channels[2])

    # mask = cv2.merge(channels)
    # mask_grey = cv2.cvtColor(cv2.cvtColor(mask, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    ## mask_grey = cv2.copyTo(img, mask)

    # [[low1, low2, low3], [up1, up2, up3]] 형태일 때
    mask_grey = cv2.inRange(img, range[0], range[1])

    return mask_grey


def detect_target(img, target_shape, draw_contour=True):
    """

    타겟 마크의 [area, center_col] 반환

    Args:
        img (np.ndarray): image only with target color
        target_shape (int): number of sides of target shape. 3, 4, or over
        draw_contour (bool): Whether to draw detected marks

    Todo:
        * findCountours 모드 좋은 걸로 탐색
        * 몇 개의 도형이 검출되었고, 몇 개가 목표에 해당하는지, 몇 개가 각각 어디에 해당하는지
        * 타겟이 다수 개 탐색되었다면 어떤 것을 목표로 할지 -> 최대 너비
        * 타겟 픽셀 위치 반환하기
        * 각 도형에 라벨 써넣기, 상자 그리기, 위치 텍스트로 쓰기, 타겟은 다른 색으로

    Note:
    Triangle
        [[[105 124]]

        [[107 205]]

        [[226 163]]]
    <type 'numpy.ndarray'>
    shape = (3, 1, 2) -> 변 개수, 1, [행, 열]인 두 개
    """
    detected = False
    morph_kernel = np.ones((9, 9), np.uint8)
    morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel)
    shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)
    targets = [] # [area, center_col]

    _, contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    
    num_contours = len(contours)
    print("# of conturs : {}".format(num_contours))

    for contour in contours:
        approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)
        
        area = cv2.contourArea(approx)
        if area < 500:
            continue

        print("-"*30)
        print("area : {}".format(area))

        vertex_num = len(approx)
        print("# of vertices : {}".format(vertex_num))

        if vertex_num == 3:
            shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
            print("Shape : Triangle")
            center_row = sum(approx[:, 0, 0]) / vertex_num
            center_col = sum(approx[:, 0, 1]) / vertex_num
            print("Center : ({}, {})".format(center_row, center_col))
            shape = cv2.circle(shape, (center_row, center_col), 2, (255, 0, 0), 2)
            if target_shape == 3:
                detected = True
                targets.append([area, center_col])
        elif vertex_num == 4:
            shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
            print("Shape : Rectangle")
            center_row = sum(approx[:, 0, 0]) / vertex_num
            center_col = sum(approx[:, 0, 1]) / vertex_num
            print("Center : ({}, {})".format(center_row, center_col))
            shape = cv2.circle(shape, (center_row, center_col), 2, (255, 0, 0), 2)
            if target_shape == 4:
                detected = True
                targets.append([area, center_col])
        else:
            _, radius = cv2.minEnclosingCircle(approx)
            ratio = radius * radius * 3.14 / area
            if ratio > 0.5 and ratio < 2:
                print("Shape : Circle")
                shape = cv2.drawContours(shape, [approx], -1, (0, 0, 255), -1)
                center_row = sum(approx[:, 0, 0]) / vertex_num
                center_col = sum(approx[:, 0, 1]) / vertex_num
                print("Center : ({}, {})".format(center_row, center_col))
                shape = cv2.circle(shape, (center_row, center_col), 2, (255, 0, 0), 2)
                if target_shape == 5:
                    detected = True
                    targets.append([area, center_col])

        cv2.imshow("Shape : shape", shape)

    if detected:
        if len(targets) == 1:
            return targets[-1]
        else:
            max_area_idx = targets.index(max(targets)) # 0번째 요소 기준 정렬
            return targets[max_area_idx]
    else:
        return None

def trackbar_callback(usrdata):
    pass

def set_HSV_value(range):
    range[0][0] = cv2.getTrackbarPos("H lower", "hsv")
    range[1][0] = cv2.getTrackbarPos("H upper", "hsv")
    range[0][1] = cv2.getTrackbarPos("S lower", "hsv")
    range[1][1] = cv2.getTrackbarPos("S upper", "hsv")
    range[0][2] = cv2.getTrackbarPos("V lower", "hsv")
    range[1][2] = cv2.getTrackbarPos("V upper", "hsv")

    return range

path_prefix = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))

raw_img = cv2.imread(path_prefix + '/pic23.jpeg', cv2.IMREAD_COLOR)
raw_img = cv2.resize(raw_img, (640, 480))
if raw_img is None:
    print("Image load failed!")
    exit(1)
cv2.imshow("win", raw_img)

cv2.namedWindow("hsv")
cv2.createTrackbar("H lower", "hsv", 0, 180, trackbar_callback)
cv2.createTrackbar("H upper", "hsv", 118, 180, trackbar_callback)
cv2.createTrackbar("S lower", "hsv", 144, 255, trackbar_callback)
cv2.createTrackbar("S upper", "hsv", 255, 255, trackbar_callback)
cv2.createTrackbar("V lower", "hsv", 72, 255, trackbar_callback)
cv2.createTrackbar("V upper", "hsv", 255, 255, trackbar_callback)

range = np.empty((2, 3))

while True:
    processed_img = preprocess_image(raw_img) #, hsv=True, blur=True
    range = set_HSV_value(range)
    mask = select_color(processed_img, range) #[[100, 200], [100, 200], [100, 200]]  np.array([[100, 0, 100], [255, 150, 255]])
    print(detect_target(mask, 3))
    
    cv2.imshow("win2", mask)
    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        break

# TODO 테스트 파일 모드를 ros 실시간 모드 혹은 이미지 모드 두 개로 선택해 구현
# imshow 할 때 창 위치 고정하기!
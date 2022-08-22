#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual

def visualize(ac):
    """Rviz visualization

    Args:
        ac (Autonomous Class): 자율운항 장애물통과 클래스
    Returns:
        all_markers (MarkerArray): Rviz로 Publish할 마커 리스트
    """
    ids = list(range(0, 100)) # marker id

    # 경기장
    boundary = visual.linelist_rviz(
        name="boundary",
        id=ids.pop(),
        lines=[
            ac.boundary[0],
            ac.boundary[1],
            ac.boundary[1],
            ac.boundary[2],
            ac.boundary[2],
            ac.boundary[3],
            ac.boundary[3],
            ac.boundary[0],
        ],
        color_r=59,
        color_g=196,
        color_b=212,
        scale=0.15,
    )

    # 목표 지점
    goal = visual.point_rviz(
        name="goal",
        id=ids.pop(),
        x=ac.goal_x,
        y=ac.goal_y,
        color_r=165,
        color_g=242,
        color_b=87,
        scale=0.2,
    )
    goal_txt = visual.text_rviz(
        name="goal",
        id=ids.pop(),
        x=ac.goal_x,
        y=ac.goal_y,
        text="({:>4.2f}, {:>4.2f})".format(ac.goal_x, ac.goal_y),
    )

    # goal_range (도착 인정 범위)
    goal_range = visual.cylinder_rviz(
        name="waypoints",
        id=ids.pop(),
        x=ac.goal_x,
        y=ac.goal_y,
        scale=ac.goal_range * 2,
        color_r=165,
        color_g=242,
        color_b=87,
    )

    # 현재 위치
    boat_txt = visual.text_rviz(
        name="boat",
        id=ids.pop(),
        text="({:>4.2f}, {:>4.2f})".format(ac.boat_x, ac.boat_y),
        x=ac.boat_x - 0.3,
        y=ac.boat_y - 0.3,
    )

    # 배로부터 목표지점까지 이은 선분
    goal_line = visual.linelist_rviz(
        name="goal_line",
        id=ids.pop(),
        lines=[[ac.boat_x, ac.boat_y], [ac.goal_x, ac.goal_y]],
        color_r=91,
        color_g=169,
        color_b=252,
        scale=0.05,
    )

    # 배와 함께 이동할 X, Y축
    axis_x = visual.linelist_rviz(
        name="axis",
        id=ids.pop(),
        lines=[[ac.boat_x, ac.boat_y], [ac.boat_x + 3, ac.boat_y]],
        color_r=255,
        scale=0.1,
    )
    axis_y = visual.linelist_rviz(
        name="axis",
        id=ids.pop(),
        lines=[[ac.boat_x, ac.boat_y], [ac.boat_x, ac.boat_y + 3]],
        color_g=255,
        scale=0.1,
    )
    axis_x_txt = visual.text_rviz(name="axis", id=14, text="X", x=ac.boat_x + 3.3, y=ac.boat_y)
    axis_y_txt = visual.text_rviz(name="axis", id=15, text="Y", x=ac.boat_x, y=ac.boat_y + 3.3)

    # 지나온 경로
    traj = visual.points_rviz(name="traj", id=ids.pop(), points=ac.trajectory, color_g=180, scale=0.05)

    # heading
    psi_arrow_end_x = 2 * math.cos(math.radians(ac.psi)) + ac.boat_x
    psi_arrow_end_y = 2 * math.sin(math.radians(ac.psi)) + ac.boat_y
    psi = visual.arrow_rviz(
        name="psi",
        id=ids.pop(),
        x1=ac.boat_x,
        y1=ac.boat_y,
        x2=psi_arrow_end_x,
        y2=psi_arrow_end_y,
        color_r=221,
        color_g=119,
        color_b=252,
    )
    psi_txt = visual.text_rviz(name="psi", id=5, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)

    # psi_desire (가고 싶은 각도)
    desire_arrow_end_x = 3 * math.cos(math.radians(ac.psi_desire)) + ac.boat_x
    desire_arrow_end_y = 3 * math.sin(math.radians(ac.psi_desire)) + ac.boat_y
    psi_desire = visual.arrow_rviz(
        name="psi_desire",
        id=ids.pop(),
        x1=ac.boat_x,
        y1=ac.boat_y,
        x2=desire_arrow_end_x,
        y2=desire_arrow_end_y,
        color_r=59,
        color_g=139,
        color_b=245,
    )
    psi_desire_txt = visual.text_rviz(
        name="psi_desire", id=ids.pop(), text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
    )

    # danger_angles
    dangers = []
    for angle in ac.danger_angles:
        end_point_x = ac.ob_dist_range * math.cos(math.radians(ac.psi + angle)) + ac.boat_x
        end_point_y = ac.ob_dist_range * math.sin(math.radians(ac.psi + angle)) + ac.boat_y
        dangers.append([ac.boat_x, ac.boat_y])
        dangers.append([end_point_x, end_point_y])
    danger_angles = visual.linelist_rviz(
        name="obs", id=ids.pop(), lines=dangers, color_r=217, color_g=217, color_b=43, color_a=100, scale=0.02
    )

    # inrange obstacles
    inrange_obs_world = []  # span 미포함
    for ob in ac.inrange_obstacles:
        begin_x = (
            ac.boat_x
            + (-ob.begin.x) * math.cos(math.radians(ac.psi))
            - ob.begin.y * math.sin(math.radians(ac.psi))
        )
        begin_y = (
            ac.boat_y
            + (-ob.begin.x) * math.sin(math.radians(ac.psi))
            + ob.begin.y * math.cos(math.radians(ac.psi))
        )
        end_x = (
            ac.boat_x
            + (-ob.end.x) * math.cos(math.radians(ac.psi))
            - ob.end.y * math.sin(math.radians(ac.psi))
        )
        end_y = (
            ac.boat_y
            + (-ob.end.x) * math.sin(math.radians(ac.psi))
            + ob.end.y * math.cos(math.radians(ac.psi))
        )
        inrange_obs_world.append([begin_x, begin_y])
        inrange_obs_world.append([end_x, end_y])
    obstacles = visual.linelist_rviz(
        name="obs", id=ids.pop(), lines=inrange_obs_world, color_r=237, color_g=234, color_b=74, scale=0.1
    )

    # angle_range (탐색 범위)
    min_angle_x = ac.ob_dist_range * math.cos(math.radians(ac.psi + ac.ob_angle_range[0])) + ac.boat_x
    min_angle_y = ac.ob_dist_range * math.sin(math.radians(ac.psi + ac.ob_angle_range[0])) + ac.boat_y
    max_angle_x = ac.ob_dist_range * math.cos(math.radians(ac.psi + ac.ob_angle_range[1])) + ac.boat_x
    max_angle_y = ac.ob_dist_range * math.sin(math.radians(ac.psi + ac.ob_angle_range[1])) + ac.boat_y
    angle_range = visual.linelist_rviz(
        name="angle_range",
        id=ids.pop(),
        lines=[
            [ac.boat_x, ac.boat_y],
            [min_angle_x, min_angle_y],
            [ac.boat_x, ac.boat_y],
            [max_angle_x, max_angle_y],
        ],
        color_r=160,
        color_g=90,
        color_b=227,
        scale=0.05,
    )

    # lidar raw data
    pcd = []
    if ac.show_raw_pcd:
        for p in ac.input_points:
            x = ac.boat_x + (-p.x) * math.cos(math.radians(ac.psi)) - p.y * math.sin(math.radians(ac.psi))
            y = ac.boat_y + (-p.x) * math.sin(math.radians(ac.psi)) + p.y * math.cos(math.radians(ac.psi))
            pcd.append([x, y])
    pcd = visual.points_rviz(name="pcd", id=ids.pop(), points=pcd, color_r=255, scale=0.08)

    all_markers = visual.marker_array_rviz(
        [
            boundary,
            goal,
            goal_txt,
            goal_range,
            boat_txt,
            traj,
            goal_line,
            axis_x,
            axis_y,
            axis_x_txt,
            axis_y_txt,
            psi,
            psi_txt,
            psi_desire,
            psi_desire_txt,
            danger_angles,
            obstacles,
            angle_range,
            pcd,
        ]
    )
    
    return all_markers
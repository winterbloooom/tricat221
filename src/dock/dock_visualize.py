#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
import math
import random

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual

def visualize(dc, psi_desire, inrange_obstacles=[], danger_angels=[]):
    """
    dc: 도킹 클래스
    """
    # 지나온 경로
    traj = visual.points_rviz(
        name="traj", id=0, points=dc.trajectory, color_g=180, scale=0.05
    )

    # heading
    psi_arrow_end_x = 2 * math.cos(math.radians(dc.psi)) + dc.boat_x
    psi_arrow_end_y = 2 * math.sin(math.radians(dc.psi)) + dc.boat_y
    psi = visual.arrow_rviz(
        name="psi",
        id=1,
        x1=dc.boat_x,
        y1=dc.boat_y,
        x2=psi_arrow_end_x,
        y2=psi_arrow_end_y,
        color_r=221,
        color_g=119,
        color_b=252,
    )
    psi_txt = visual.text_rviz(
        name="psi", id=2, text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y
    )

    # psi_desire (가고 싶은 각도)
    desire_arrow_end_x = 3 * math.cos(math.radians(psi_desire)) + dc.boat_x
    desire_arrow_end_y = 3 * math.sin(math.radians(psi_desire)) + dc.boat_y
    desire = visual.arrow_rviz(
        name="psi_desire",
        id=3,
        x1=dc.boat_x,
        y1=dc.boat_y,
        x2=desire_arrow_end_x,
        y2=desire_arrow_end_y,
        color_r=59,
        color_g=139,
        color_b=245,
    )
    desire_txt = visual.text_rviz(
        name="psi_desire", id=4, text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
    )

    # 배와 함께 이동할 X, Y축
    axis_x = visual.linelist_rviz(
        name="axis",
        id=5,
        lines=[[dc.boat_x, dc.boat_y], [dc.boat_x + 3, dc.boat_y]],
        color_r=255,
        scale=0.1,
    )
    axis_y = visual.linelist_rviz(
        name="axis",
        id=6,
        lines=[[dc.boat_x, dc.boat_y], [dc.boat_x, dc.boat_y + 3]],
        color_g=255,
        scale=0.1,
    )
    axis_x_txt = visual.text_rviz(
        name="axis", id=7, text="X", x=dc.boat_x + 3.3, y=dc.boat_y
    )
    axis_y_txt = visual.text_rviz(
        name="axis", id=8, text="Y", x=dc.boat_x, y=dc.boat_y + 3.3
    )

    all_markers = visual.marker_array_rviz([traj, psi, psi_txt, desire, desire_txt, axis_x, axis_y, axis_x_txt, axis_y_txt])



    # =========================================== apend 시키기
    
    if dc.state == 0:
        # angle_range (탐색 범위)
        min_angle_x = (
            dc.ob_dist_range * math.cos(math.radians(dc.psi + dc.ob_angle_range[0]))
            + dc.boat_x
        )
        min_angle_y = (
            dc.ob_dist_range * math.sin(math.radians(dc.psi + dc.ob_angle_range[0]))
            + dc.boat_y
        )
        max_angle_x = (
            dc.ob_dist_range * math.cos(math.radians(dc.psi + dc.ob_angle_range[1]))
            + dc.boat_x
        )
        max_angle_y = (
            dc.ob_dist_range * math.sin(math.radians(dc.psi + dc.ob_angle_range[1]))
            + dc.boat_y
        )
        angle_range = visual.linelist_rviz(
            name="angle_range",
            id=9,
            lines=[
                [dc.boat_x, dc.boat_y],
                [min_angle_x, min_angle_y],
                [dc.boat_x, dc.boat_y],
                [max_angle_x, max_angle_y],
            ],
            color_r=160,
            color_g=90,
            color_b=227,
            scale=0.05,
        )
        visual.marker_array_append_rviz(all_markers, angle_range)

        # danger angles
        dangers = []
        for angle in danger_angels:
            end_point_x = (
                dc.ob_dist_range * math.cos(math.radians(dc.psi + angle)) + dc.boat_x
            )
            end_point_y = (
                dc.ob_dist_range * math.sin(math.radians(dc.psi + angle)) + dc.boat_y
            )
            dangers.append([dc.boat_x, dc.boat_y])
            dangers.append([end_point_x, end_point_y])
        danger_ang = visual.linelist_rviz(
            name="obs",
            id=10,
            lines=dangers,
            color_r=217,
            color_g=217,
            color_b=43,
            color_a=100,
            scale=0.02,
        )
        visual.marker_array_append_rviz(all_markers, danger_ang)

        # inrange obs
        inrange_obs_world = []  # span 미포함
        for ob in inrange_obstacles:
            begin_x = (
                dc.boat_x
                + (-ob.begin.x) * math.cos(math.radians(dc.psi))
                - ob.begin.y * math.sin(math.radians(dc.psi))
            )
            begin_y = (
                dc.boat_y
                + (-ob.begin.x) * math.sin(math.radians(dc.psi))
                + ob.begin.y * math.cos(math.radians(dc.psi))
            )
            end_x = (
                dc.boat_x
                + (-ob.end.x) * math.cos(math.radians(dc.psi))
                - ob.end.y * math.sin(math.radians(dc.psi))
            )
            end_y = (
                dc.boat_y
                + (-ob.end.x) * math.sin(math.radians(dc.psi))
                + ob.end.y * math.cos(math.radians(dc.psi))
            )
            inrange_obs_world.append([begin_x, begin_y])
            inrange_obs_world.append([end_x, end_y])
        obstacles = visual.linelist_rviz(
            name="obs",
            id=11,
            lines=inrange_obs_world,
            color_r=237,
            color_g=234,
            color_b=74,
            scale=0.1,
        )
        visual.marker_array_append_rviz(all_markers, obstacles)

    if dc.state in [0, 1, 2, 3]:
        ids = random.sample(range(20,40),9)

        # 배로부터 목표지점까지 이은 선분
        goal_line = visual.linelist_rviz(
            name="goal_line",
            id=ids.pop(),
            lines=[[dc.boat_x, dc.boat_y], dc.waypoints[dc.state]],
            color_r=91,
            color_g=169,
            color_b=252,
            scale=0.05,
        )
        visual.marker_array_append_rviz(all_markers, goal_line)

        for i in range(4):
            x_pos = dc.waypoints[i][0]
            y_pos = dc.waypoints[i][1]
            # 도착 인정 범위
            waypoint = visual.cylinder_rviz(
                name="waypoints",
                id=ids.pop(),
                x=x_pos,
                y=y_pos,
                scale=dc.arrival_range * 2,
                color_r=165,
                color_g=242,
                color_b=87,
            )
            visual.marker_array_append_rviz(all_markers, waypoint)

            # 좌표
            txt = visual.text_rviz(
                name="waypoints",
                id=ids.pop(),
                x=x_pos,
                y=y_pos,
                text="#{} ({:>4.2f}, {:>4.2f})".format(i, x_pos, y_pos),
            )
            visual.marker_array_append_rviz(all_markers, txt)

    if not dc.state == 0:
        # station 방향
        station_arrow_end_x = 2 * math.cos(math.radians(dc.station_dir)) + dc.boat_x
        station_arrow_end_y = 2 * math.sin(math.radians(dc.station_dir)) + dc.boat_y
        station = visual.arrow_rviz(
            name="station",
            id=12,
            x1=dc.boat_x,
            y1=dc.boat_y,
            x2=station_arrow_end_x,
            y2=station_arrow_end_y,
            color_r=65,
            color_g=53,
            color_b=240,
        )
        visual.marker_array_append_rviz(all_markers, station)

        station_txt = visual.text_rviz(
            name="station", id=16, text="station", x=station_arrow_end_x, y=station_arrow_end_y
        )
        visual.marker_array_append_rviz(all_markers, station_txt)

    if dc.state == 4:
        # heading 회전 인정 범위
        min_angle_x = 3 * math.cos(math.radians(dc.station_dir - dc.ref_dir_range)) + dc.boat_x
        min_angle_y = 3 * math.sin(math.radians(dc.station_dir - dc.ref_dir_range)) + dc.boat_y
        max_angle_x = 3 * math.cos(math.radians(dc.station_dir + dc.ref_dir_range)) + dc.boat_x
        max_angle_y = 3 * math.sin(math.radians(dc.station_dir + dc.ref_dir_range)) + dc.boat_y
        heading_range = visual.linelist_rviz(
            name="heading_range",
            id=13,
            lines=[
                [dc.boat_x, dc.boat_y],
                [min_angle_x, min_angle_y],
                [dc.boat_x, dc.boat_y],
                [max_angle_x, max_angle_y],
            ],
            color_r=122,
            color_g=114,
            color_b=237,
        )
        visual.marker_array_append_rviz(all_markers, heading_range)

    return all_markers
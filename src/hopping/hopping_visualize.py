#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import utils.visualizer as visual


def visualize(hc):
    """Rviz visualization

    Args:
        hc (Hopping Class): 호핑투어 클래스
    Returns:
        all_markers (MarkerArray): Rviz로 Publish할 마커 리스트
    """
    ids = list(range(0, 100))  # marker id

    # 현재 보트 위치 좌표
    boat = visual.text_rviz(
        name="boat",
        id=ids.pop(),
        text="({:>4.2f}, {:>4.2f})".format(hc.boat_x, hc.boat_y),
        x=hc.boat_x - 0.3,
        y=hc.boat_y - 0.3,
    )

    # 이동 궤적
    traj = visual.points_rviz(name="traj", id=ids.pop(), points=hc.trajectory, color_g=255)

    # 선수각
    psi_arrow_end_x = 2 * math.cos(math.radians(hc.psi)) + hc.boat_x
    psi_arrow_end_y = 2 * math.sin(math.radians(hc.psi)) + hc.boat_y
    psi = visual.arrow_rviz(
        name="psi",
        id=ids.pop(),
        x1=hc.boat_x,
        y1=hc.boat_y,
        x2=psi_arrow_end_x,
        y2=psi_arrow_end_y,
        color_r=221,
        color_g=119,
        color_b=252,
    )
    psi_txt = visual.text_rviz(name="psi", id=ids.pop(), text="psi", x=psi_arrow_end_x, y=psi_arrow_end_y)

    # psi_desire (가고 싶은 각도)
    desire_arrow_end_x = 3 * math.cos(math.radians(hc.psi_desire)) + hc.boat_x
    desire_arrow_end_y = 3 * math.sin(math.radians(hc.psi_desire)) + hc.boat_y
    psi_desire = visual.arrow_rviz(
        name="psi_desire",
        id=ids.pop(),
        x1=hc.boat_x,
        y1=hc.boat_y,
        x2=desire_arrow_end_x,
        y2=desire_arrow_end_y,
        color_r=59,
        color_g=139,
        color_b=245,
    )
    psi_desire_txt = visual.text_rviz(
        name="psi_desire", id=ids.pop(), text="desire", x=desire_arrow_end_x, y=desire_arrow_end_y
    )

    # 배로부터 목표지점까지 이은 선분
    goal_line = visual.linelist_rviz(
        name="goal_line",
        id=ids.pop(),
        lines=[[hc.boat_x, hc.boat_y], [hc.goal_x, hc.goal_y]],
        color_r=91,
        color_g=169,
        color_b=252,
        scale=0.05,
    )

    # 배와 함께 이동할 X, Y축
    axis_x = visual.linelist_rviz(
        name="axis_x",
        id=100,
        lines=[[hc.boat_x, hc.boat_y], [hc.boat_x + 3, hc.boat_y]],
        color_r=255,
        scale=0.1,
    )
    axis_y = visual.linelist_rviz(
        name="axis_x",
        id=ids.pop(),
        lines=[[hc.boat_x, hc.boat_y], [hc.boat_x, hc.boat_y + 3]],
        color_g=255,
        scale=0.1,
    )
    axis_x_txt = visual.text_rviz(name="axis", id=ids.pop(), text="X", x=hc.boat_x + 3.3, y=hc.boat_y)
    axis_y_txt = visual.text_rviz(name="axis", id=ids.pop(), text="Y", x=hc.boat_x, y=hc.boat_y + 3.3)

    all_markers = visual.marker_array_rviz(
        [
            psi,
            boat,
            psi_txt,
            traj,
            psi_desire,
            psi_desire_txt,
            goal_line,
            axis_x,
            axis_y,
            axis_x_txt,
            axis_y_txt,
        ]
    )

    # 남은 waypoints
    for idx in range(hc.waypoint_idx, len(hc.waypoints) + 1):
        # waypoints
        waypoint = visual.point_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.waypoints[idx][0],
            y=hc.waypoints[idx][1],
            color_r=78,
            color_g=166,
            color_b=58,
            scale=0.3,
        )
        visual.marker_array_append_rviz(all_markers, waypoint)
        waypoint_txt = visual.text_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.remained_waypoints[idx][0],
            y=hc.remained_waypoints[idx][1],
            text=str(idx),
            scale=1.2,
        )
        visual.marker_array_append_rviz(all_markers, waypoint_txt)

        # waypoints의 도착 인정 범위
        goal_range = visual.cylinder_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.remained_waypoints[idx][0],
            y=hc.remained_waypoints[idx][1],
            scale=hc.goal_range * 2,
            color_r=78,
            color_g=166,
            color_b=58,
        )
        visual.marker_array_append_rviz(all_markers, goal_range)

    # 통과한 waypoints
    for idx in range(1, hc.waypoint_idx):
        # waypoints
        waypoint = visual.point_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.waypoints[idx][0],
            y=hc.waypoints[idx][1],
            color_r=66,
            color_g=135,
            color_b=245,
            scale=0.3,
        )
        visual.marker_array_append_rviz(all_markers, waypoint)
        waypoint_txt = visual.text_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.waypoints[idx][0],
            y=hc.waypoints[idx][1],
            text=str(idx),
            scale=1.2,
        )
        visual.marker_array_append_rviz(all_markers, waypoint_txt)

        # waypoints의 도착 인정 범위
        goal_range = visual.cylinder_rviz(
            name="waypoints",
            id=ids.pop(),
            x=hc.waypoints[idx][0],
            y=hc.waypoints[idx][1],
            scale=hc.goal_range * 2,
            color_r=66,
            color_g=135,
            color_b=245,
        )
        visual.marker_array_append_rviz(all_markers, goal_range)

    return all_markers

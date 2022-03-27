"""
시작, ObstacleAvoidance 호출 후


    while not autonomous.is_all_connected():
        rospy.sleep(0.2)

    while not autonomous.arrival_check():
        autonomous.calc_angle_risk()
        autonomous.control_publish()
        autonomous.print_state()
        autonomous.view_rviz()
        rate.sleep()
    
도착했음 -> 도킹으로 전환

StartboardCam 호출, mark_detect() 수행, 천천히 전진
    True가 나오면 p1, p2 계산
        BowCam 호출
    False이면 끝 지점에 도착했는지 확인
        도착했으면 탐지 실패한 것. 후진해서 시작점으로 이동
        도착하지 않았으면 계속 함수 돌리며 전진

BowCam 호출, error_to_middle() 수행, 마크 우선 서보 모터 제어량 결정
Lidar로 장애물 계산, 스테이션 우선 서보 모터 제어량 결정
 -> 중앙에서 가장 가까운 점들의 집합을 돌출부라 가정
 -> 돌출부의 위치가 중앙을 기준으로 양 옆에 있고, 배가 지나갈 수 있는 각도에 거리 확보되어 있다면 괜찮
 -> 거리가 너무 가까우면 회피 우선
서보 모터 제어량 중 어떻게 우선순위 두어 제어해야 할 지 결정
퍼블리시
"""


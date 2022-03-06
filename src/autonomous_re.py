        self.inrange_obstacle = []

    def calc_ob_risk(self):
        dist_to_obstacles = [] # [장애물 중점까지 거리, 각도 인덱스]의 배열

        for ob in self.obstacle:
            begin_ang = math.degrees(atan2(ob.begin.y, ob.begin.x)) # 장애물 블럭 시작점
            end_ang = math.degrees(atan2(ob.end.y, ob.end.x))   # 장애물 블럭 끝점

            middle_x = (ob.begin.x + ob.end.x) / 2  # 장애물 블럭 중점
            middle_y = (ob.begin.y + ob.end.y) / 2
            middle_ang = round(math.degrees(atan2(middle_y, middle_x))) # TODO increment를 1로 했을 때를 가정해 round를 했음 소숫점까지 내려가면 모르겠다..

            if middle_ang < self.angle_min or middle_ang > self.angle_max:
                continue  # min_angle, max_angle 벗어난 것 처리함. TODO 잘 들어가는지 확인
            else:
                self.inrange_obstacle.append(ob)    # 계산 범위에 있는 장애물임

            if (begin_ang - self.span_angle) > self.angle_min:
                start_ang = round(begin_ang - self.span_angle)
            else:
                start_ang = self.angle_min
            
            if (end_ang + self.span_angle) > self.angle_max:
                fin_ang = round(end_ang + self.span_angle)
            else:
                fin_ang = self.angle_max
            
            start_ang_idx = (start_ang - self.angle_min) / self.angle_increment
            fin_ang_idx = (fin_ang - self.angle_min) / self.angle_increment
            middle_ang_idx = int((start_ang_idx + fin_ang_idx) / 2) # 장애물이 계산 범위 넘어가 있어 start_ang / fin_ang이 바뀌었더라도 탐지범위 내 있는 크기의 중간으로 삼음

            for idx in range(start_ang_idx, end_ang_idx + 1, self.angle_increment):
                self.angle_risk[idx] += self.ob_exist_coefficient  # TODO 잘 작동하는지 확인할 것

            dist = math.sqrt(pow(self.boat_x - middle_x, 2.0) + pow(self.boat_y - middle_y, 2.0))   # 장애물 중점에서 배까지 거리
            self.angle_risk[middle_ang_idx] += dist * self.ob_near_coefficient  # 거리에 비례해 곱해줌. 때문에 해당 계수는 1 이하여야함


    def calc_goal_risk(self): # 목표점에 가까워야 하니까 위험도는 낮아짐.
        error_goal_ang = self.psi_goal - self.psi # 선박고정좌표계 기준 도착지까지 각도

        if self.angle_min <= error_goal_ang <= self.angle_max: # 계산 범위 안에 목표점이 존재함
            for idx in range(len(self.angle_risk)):
                ang = idx * self.angle_increment + self.angle_min
                delta_ang = abs(error_goal_ang - ang) # TODO 값 확인하기
                if delta_ang <= self.goal_risk_range:
                    self.angle_risk[idx] += 1 * self.goal_orient_coefficient   # 해당 각도가 목표점과 많이 떨어져 있지 않음. 값을 조금만 더해줌
                elif delta_ang <= (self.goal_risk_range * 2):
                    self.angle_risk[idx] += 2 * self.goal_orient_coefficient
                elif delta_ang <= (self.goal_risk_range * 3):
                    self.angle_risk[idx] += 3 * self.goal_orient_coefficient
                else:
                    self.angle_risk[idx] += 4 * self.goal_orient_coefficient
        elif self.angle_min > error_goal_ang: # 목적지가 탐색 범위를 벗어난 좌현에 위치 -> TODO 일단 중심 각도를 기준으로 우쪽에 모두 점수 부여. 반대쪽이니까 우현에 안 가도록 위험도 높임
            for idx in range(len(self.angle_risk)//2, len(self.angle_risk) + 1):
                self.angle_risk[idx] += 2 * self.goal_orient_coefficient # TODO 2를 곱하는 게 맞을까?
        else:   # 목적지가 탐색 범위를 벗어난 우현에 위치(ex. 출발 지점) -> TODO 일단 중심 각도를 기준으로 왼쪽에 모두 점수 부여
            for idx in range(0, len(self.angle_risk)//2 + 1):
                self.angle_risk[idx] += 2 * self.goal_orient_coefficient # TODO 1을 곱하는 게 맞을까?
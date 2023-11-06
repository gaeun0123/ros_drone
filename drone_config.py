import numpy as np
import rospy

def expand_obstacles(obstacles, padding=2, grid_size=200):
    expanded_obstacles = set()
    for (x, y) in obstacles:
        for i in range(-padding, padding+1):
            for j in range(-padding, padding+1):
                new_x = x + i + grid_size // 2  # 중앙 기준으로 조정
                new_y = y + j + grid_size // 2  # 중앙 기준으로 조정
                if 0 <= new_x < grid_size and 0 <= new_y < grid_size:  # 좌표가 유효한지 확인
                    expanded_obstacles.add((new_x, new_y))
    return list(expanded_obstacles)

def expand_maze_zero_filled(obstacles, grid_size=200):
    expanded_obstacles = expand_obstacles(obstacles, grid_size=grid_size)
    maze = np.zeros((grid_size, grid_size), dtype=int)  # 그리드 크기 변경
    
    for (x, y) in expanded_obstacles:
        maze[y, x] = 1
    
    return maze

# 장애물의 위치를 중심 기준 좌표계로 변환
def adjust_obstacles_to_center(obstacles, grid_size=200):
    return [(x + grid_size // 2, y + grid_size // 2) for (x, y) in obstacles]

# 장애물의 위치 좌표 리스트
obstacles = [(16,16),(16,17),(16,18),(16,19),(16,20),(16,21),(16,22),(16,23),(16,24),(16,25),
	      (17,16),(17,17),(17,18),(17,19),(17,20),(17,21),(17,22),(17,23),(17,24),(17,25),
	      (18,16),(18,17),(18,18),(18,19),(18,20),(18,21),(18,22),(18,23),(18,24),(18,25),
	      (19,16),(19,17),(19,18),(19,19),(19,20),(19,21),(19,22),(19,23),(19,24),(19,25),
	      (20,16),(20,17),(20,18),(20,19),(20,20),(20,21),(20,22),(20,23),(20,24),(20,25),
	      (21,16),(21,17),(21,18),(21,19),(21,20),(21,21),(21,22),(21,23),(21,24),(21,25),
	      (22,16),(22,17),(22,18),(22,19),(22,20),(22,21),(22,22),(22,23),(22,24),(22,25),
	      (23,16),(23,17),(23,18),(23,19),(23,20),(23,21),(23,22),(23,23),(23,24),(23,25),
	      (24,16),(24,17),(24,18),(24,19),(24,20),(24,21),(24,22),(24,23),(24,24),(24,25),
	      (25,16),(25,17),(25,18),(25,19),(25,20),(25,21),(25,22),(25,23),(25,24),(25,25)]  # 장애물 좌표

# 중앙 기준으로 장애물 좌표 조정
center_adjusted_obstacles = adjust_obstacles_to_center(obstacles, grid_size=200)
maze_200x200 = expand_maze_zero_filled(center_adjusted_obstacles, grid_size=200)

drone_configs = {
    "uav0": {
        "maze": maze_200x200,
        "start": (0, 0),
        "end": (90, 90)
    },
    "uav1": {
        "maze": maze_200x200,
        "start": (0, 0),
        "end": (-10, -10)
    }
}
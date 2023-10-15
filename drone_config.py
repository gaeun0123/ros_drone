import numpy as np

def expand_maze_zero_filled():
    maze_10x10 = np.zeros((100, 100), dtype=int)  # 10x10 크기의 0으로 채워진 배열 생성

    # 각 셀을 2x2 크기로 확장하여 20x20 크기의 배열 생성
    maze_20x20 = np.kron(maze_10x10, np.ones((2, 2), dtype=int))
    
    return maze_20x20.tolist()  # 리스트 형태로 반환

maze_20x20 = expand_maze_zero_filled()

# 새로운 20x20 maze를 drone_configs에 추가
drone_configs = {
    "uav0": {
        "maze": maze_20x20
    },
    "uav1": {
        "maze": maze_20x20
    }
}

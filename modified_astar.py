#!/usr/bin/env python3

import rospy
from std_msgs.msg import String 
from drone_config import drone_configs as config
import drone_config 
from math import sqrt

CRITICAL = 5.0

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# 설정 경로 시각화 함수
def print_grid(grid, closed_list, path):
    for i in range(len(grid)):
        row = ""
        for j in range(len(grid[i])):
            if (i, j) in closed_list:
                row += "C "  # C represents closed nodes
            elif (i, j) in path:
                row += "P "  # P represents path nodes
            else:
                row += str(grid[i][j]) + " "
        print(row)
    print("\n")

# 드론 간 거리 계산 함수
def calculate_distance(point1, point2):
    return sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

# 임계 구역에 따른 maze 수정 함수
def set_critical_areas(maze, drone_positions, current_pose, critical_distance):
    critical_areas = []
    # current_pose : Pose 타입의 객체
    current_pose = (current_pose.position.x, current_pose.position.y)

    for _, other_pose in drone_positions.values():
        # other_pose : Pose 타입의 객체
        other_position = (other_pose.position.x, other_pose.position.y)
        
        if calculate_distance(current_pose.position, other_pose.position) < critical_distance:
            x, y = int(other_position[0]), int(other_position[1])
            if 0 <= x < 200 and 0 <= y < 200:  # 미로 범위 체크
                maze[y][x] = 1  # 장애물 표시, [y][x]=(x,y)
                # 임계 구역에 추가
                critical_areas.append((y, x))
    return maze

# 경로 재계산 함수
def recalculation_path(namespace, current_position, other_drone_positions):
    namespace = namespace.strip('/')

    uav_config = config.get(namespace)

    if not uav_config:
        raise KeyError(f"다음과 같은 namespace uav가 없습니다. {namespace}")
    
    # 출발지 빼고 기존 정보 유지
    start = (int(current_position.x), int(current_position.y))
    end = uav_config["end"]
    maze = uav_config["maze"]

    # 임계 구역 설정
    maze = set_critical_areas(maze, other_drone_positions, current_position, CRITICAL)

    # 수정된 위치로부터 경로 재계산
    new_path = astar(maze, start, end)
    return new_path

# 경로 알고리즘
def astar(grid, start, end):
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []
    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = open_list[0] 
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid) - 1]) - 1) or node_position[1] < 0:
            #     continue
            
            if 0 <= node_position[0] < len(grid) and 0 <= node_position[1] < len(grid[0]):
                if grid[node_position[0]][node_position[1]] != 0:
                    continue

            if grid[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            if len([open_node for open_node in open_list if child.position == open_node.position and child.g >= open_node.g]) > 0:
                continue

            open_list.append(child)

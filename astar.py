# Time Complexity는 H에 따라 다르다.
# O(b^d), where d = depth, b = 각 노드의 하위 요소 수
# heapque를 이용하면 길을 출력할 때 reverse를 안해도 됨
from drone_config import drone_configs
import rospy

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def heuristic(node, goal, D=1, D2=2 ** 0.5):  # Diagonal Distance
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

# 글로벌 위치 -> 그리드 위치 변환 함수
def global_position_to_grid_cell(global_position, grid_size):
    x, y = global_position
    cell_x = int(x / grid_size)
    cell_y = int(y / grid_size)
    return (cell_x, cell_y)

# 그리드 위치 -> 글로벌 위치 변환 함수
# 필요 이유 : 웨이포인트 변환
def grid_cell_to_global_position(grid_cell, grid_size):
    cell_x, cell_y = grid_cell
    x = cell_x * grid_size
    y = cell_y * grid_size
    return (x, y)

def aStar(maze, start_global, end_global):
    grid_size = 0.0001

    # startNode와 endNode 초기화
    start = global_position_to_grid_cell(start_global, grid_size)
    end = global_position_to_grid_cell(end_global, grid_size)

    # startNode와 endNode 초기화
    startNode = Node(None, start)
    endNode = Node(None, end)

    # openList, closedList 초기화
    openList = []
    closedList = []

    # openList에 시작 노드 추가
    openList.append(startNode)

    # endNode를 찾을 때까지 실행
    while openList:

        # 현재 노드 지정
        currentNode = openList[0]
        currentIdx = 0

        # 이미 같은 노드가 openList에 있고, f 값이 더 크면
        # currentNode를 openList안에 있는 값으로 교체
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentNode = item
                currentIdx = index

        # openList에서 제거하고 closedList에 추가
        openList.pop(currentIdx)
        closedList.append(currentNode)

        # 현재 노드가 목적지면 current.position 추가하고
        # current의 부모로 이동
        if currentNode == endNode:
            grid_path = []
            current = currentNode
            while current is not None:
                # maze 길을 표시하려면 주석 해제
                # x, y = current.position
                # maze[x][y] = 7 
                grid_path.append(current.position)
                current = current.parent
            
            global_path = [grid_cell_to_global_position(cell, grid_size) for cell in grid_path]
            return global_path[::-1]  # reverse

        children = []
        # 인접한 xy좌표 전부
        for newPosition in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            # 노드 위치 업데이트
            nodePosition = (
                currentNode.position[0] + newPosition[0],  # X
                currentNode.position[1] + newPosition[1])  # Y
                
            # 미로 maze index 범위 안에 있어야함
            within_range_criteria = [
                nodePosition[0] > (len(maze) - 1),
                nodePosition[0] < 0,
                nodePosition[1] > (len(maze[len(maze) - 1]) - 1),
                nodePosition[1] < 0,
            ]

            if any(within_range_criteria):  # 하나라도 true면 범위 밖임
                continue

            # 장애물이 있으면 다른 위치 불러오기
            if maze[nodePosition[0]][nodePosition[1]] != 0:
                continue

            new_node = Node(currentNode, nodePosition)
            children.append(new_node)

        # 자식들 모두 loop
        for child in children:

            # 자식이 closedList에 있으면 continue
            if child in closedList:
                continue

            # f, g, h값 업데이트
            child.g = currentNode.g + 1
            # child.h = ((child.position[0] - endNode.position[0]) **
            #            2) + ((child.position[1] - endNode.position[1]) ** 2)
            child.h = heuristic(child, endNode)
            # print("position:", child.position) 거리 추정 값 보기
            # print("from child to goal:", child.h)
            
            child.f = child.g + child.h

            # 자식이 openList에 있으고, g값이 더 크면 continue
            if len([openNode for openNode in openList
                    if child == openNode and child.g > openNode.g]) > 0:
                continue
                    
            openList.append(child)


def main(namespace, start_global, end_global):
    # drone_config.py에는 슬래쉬가 없는 채로 저장되기 때문에
    namespace = namespace.strip('/')
    
    if namespace not in drone_configs:
        raise ValueError(f"No configuration found for {namespace}")

    config = drone_configs[namespace]
    maze = config["maze"]

    path = aStar(maze, start_global, end_global)
    return path

# if __name__ == '__main__':
#     test_namespace = "uav0"
    
#     start_position = (37.7749, -122.4194)
#     end_position = (34.0522, -118.2437)
    
#     result_path = main(test_namespace, start_position, end_position)
#     print(result_path)

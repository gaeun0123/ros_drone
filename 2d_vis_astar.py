# Time Complexity는 H에 따라 다르다.
# O(b^d), where d = depth, b = 각 노드의 하위 요소 수
# heapque를 이용하면 길을 출력할 때 reverse를 안해도 됨
import matplotlib.pyplot as plt
import numpy as np

def draw_maze(maze, path=None):
    """
    미로와 경로를 시각화하는 함수
    """
    maze_copy = np.array(maze)

    # 경로를 미로에 표시합니다. (2로 표시)
    if path:
        for step in path:
            maze_copy[step[0]][step[1]] = 2

    fig, ax = plt.subplots(figsize=(8, 8))
    cmap = plt.get_cmap('tab20b')
    ax.imshow(maze_copy, cmap=cmap)
    
    # 벽을 검은색으로, 빈 공간을 흰색으로, 경로를 파란색으로 표시합니다.
    colors = {0: 'white', 1: 'black', 2: 'blue'}
    for y in range(maze_copy.shape[0]):
        for x in range(maze_copy.shape[1]):
            ax.add_patch(plt.Rectangle((x, y), 1, 1, color=colors[maze_copy[y, x]]))
            if (y, x) == path[0]:
                ax.text(x + 0.5, y + 0.5, 'S', ha='center', va='center')
            elif (y, x) == path[-1]:
                ax.text(x + 0.5, y + 0.5, 'E', ha='center', va='center')

    ax.set_xticks(np.arange(maze_copy.shape[1]))
    ax.set_yticks(np.arange(maze_copy.shape[0]))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True)
    plt.show()

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


def aStar(maze, start, end):
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
            path = []
            current = currentNode
            while current is not None:
                # maze 길을 표시하려면 주석 해제
                # x, y = current.position
                # maze[x][y] = 7 
                path.append(current.position)
                current = current.parent
            return path[::-1]  # reverse

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
            child.h = ((child.position[0] - endNode.position[0]) **
                       2) + ((child.position[1] - endNode.position[1]) ** 2)
            # child.h = heuristic(child, endNode) 다른 휴리스틱
            # print("position:", child.position) 거리 추정 값 보기
            # print("from child to goal:", child.h)
            
            child.f = child.g + child.h

            # 자식이 openList에 있으고, g값이 더 크면 continue
            if len([openNode for openNode in openList
                    if child == openNode and child.g > openNode.g]) > 0:
                continue
                    
            openList.append(child)


def main():
    maze = [
    [0, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    end = (3, 4)

    path = aStar(maze, start, end)
    print(path)
    
    draw_maze(maze,path)


if __name__ == '__main__':
    main()
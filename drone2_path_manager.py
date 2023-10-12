#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import astar
import math

# Grid 변환 관련 변수 설정
resolution = 1.0  # 예: 1m, 실제 환경에 맞게 조정
origin_x, origin_y = 0, 0  # 그리드 원점의 월드 좌표
threshold_distance = 5  # 임계값, 이 내부에 들어오면 경로 재설정

def world_to_grid(world_x, world_y):
    grid_x = int((world_x - origin_x) / resolution)
    grid_y = int((world_y - origin_y) / resolution)
    return grid_x, grid_y

# 거리 계산 함수
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

rospy.init_node('drone2_pose_listener')

# drone1의 공유 토픽을 구독
rospy.Subscriber("/drone1/shared_pose", PoseStamped, drone1_pose_callback)

# 토픽을 받으면 drone1_pose_callback함수 실행
rospy.spin()

drone2_pose = rospy.wait_for_message("/uav1/mavros/local_position/pose", PoseStamped).pose

# drone2와 drone1과의 거리 차이 구하기
distance = get_distance(drone2_pose, drone1_pose)




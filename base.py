#!/usr/bin/env python3

# qgc에서 waypoint 지정 후 waypoint를 받아 경로에 Astar 알고리즘 입히는 코드
# global position 기반

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix

class MultiDroneNode:
    def __init__(self, drone_namespace):
        self.namespace = drone_namespace

        # 상태 및 위치 정보 초기화
        self.current_state = None
        self.current_global_pose = NavSatFix()

        # Subscriber 및 Publisher 초기화
        self.state_sub = rospy.Subscriber(self.namespace + '/mavros/state', State, self.state_cb)
        self.global_pose_sub = rospy.Subscriber(self.namespace + '/mavros/global_position/global', NavSatFix, self.global_pose_cb)
        self.global_pos_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

    def state_cb(self, msg):
        self.current_state = msg

    def global_pose_cb(self, msg):
        self.current_global_pose = msg

# 실행 파일로 실행될 경우 name은 main으로 설정
# import 파일로 사용되어질 경우에는 name이 해당 스크립트 파일 명으로 변경

if __name__ == '__main__':
    # ROS 노드 초기화
    # multi_drone_node라는 노드 생성
    rospy.init_node('multi_drone_node', anonymous=True)

    # namespace를 파라미터로 받기
    drone_namespaces = rospy.get_param("~namespaces", "").split(',')

    # 각 드론의 노드 객체를 저장할 리스트 초기화
    drone_nodes = []

    # 각 드론에 대해 노드 객체 생성
    for ns in drone_namespaces:
        drone_nodes.append(MultiDroneNode(ns.strip()))

    # ROS 노드를 실행 상태로 유지하며 메시지 계속 수신하도록 함
    rospy.spin()

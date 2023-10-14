#!/usr/bin/env python3

# qgc에서 waypoint 지정 후 waypoint를 받아 경로에 Astar 알고리즘 입히는 코드
# global position 기반

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from astar import main as astar_main
import dp
from dp import douglas_peucker
from drone_config import drone_configs

class MultiDroneNode:
    def __init__(self, drone_namespace):
        self.namespace = drone_namespace

        # 상태 및 위치 정보 초기화
        self.current_state = None
        self.current_global_pose = NavSatFix()

        # waypoint를 저장하기 위한 리스트
        self.waypoints = []
        self.path = []

        # Subscriber 및 Publisher 초기화
        self.state_sub = rospy.Subscriber(self.namespace + '/mavros/state', State, self.state_cb)
        self.global_pose_sub = rospy.Subscriber(self.namespace + '/mavros/global_position/global', NavSatFix, self.global_pose_cb)
        self.global_pos_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

        self.waypoints_sub = rospy.Subscriber(self.namespace + '/mavros/mission/waypoints', WaypointList, self.waypoint_cb)
        self.arming_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        self.land_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)

    def state_cb(self, msg):
        self.current_state = msg

    def global_pose_cb(self, msg):
        self.current_global_pose = msg

    # 웨이포인트 정보 받기
    def waypoint_cb(self, data):
        self.waypoints = data.waypoints

    def takeoff(self, altitude=5.0):
        self.set_mode("GUIDED")
        self.arm()
        takeoff_cmd = CommandTOL()
        takeoff_cmd.altitude = altitude
        self.takeoff_client(takeoff_cmd)

    def land(self):
        self.set_mode("LAND")

    def set_mode(self, mode):
        rospy.wait_for_service(self.namespace + '/mavros/set_mode')
        try:
            self.set_mode_client(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def arm(self):
        rospy.wait_for_service(self.namespace + '/mavros/cmd/arming')
        try:
            self.arming_client(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def plan_path_with_astar(self):
        start_global = (self.waypoints[0].x_lat, self.waypoints[0].y_long)
        end_global = (self.waypoints[-1].x_lat, self.waypoints[-1].y_long)

        # 웨이포인트의 첫 지점과 마지막 지점을 출발지, 목적지로 설정 후 astar 알고리즘 경로 찾기
        self.path = astar_main(self.namespace, start_global, end_global)

    def publish_waypoints(self):
        rate = rospy.Rate(10)  # 10hz
        for position in self.path:
            waypoint = GlobalPositionTarget()
            waypoint.latitude = position[0]
            waypoint.longitude = position[1]

            self.global_pos_pub.publish(waypoint)
            rate.sleep()  # 10Hz로 웨이포인트 발행

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

    for drone_node in drone_nodes :
        drone_node.takeoff()
        rospy.sleep(5)  # takeoff 후에 약간의 대기 시간
        drone_node.plan_path_with_astar()
        drone_node.publish_waypoints()
    
    # ROS 노드를 실행 상태로 유지하며 메시지 계속 수신하도록 함
    rospy.spin()

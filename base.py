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
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

class MultiDroneNode:
    def __init__(self, drone_namespace):
        self.namespace = drone_namespace

        # 상태 및 위치 정보 초기화
        self.current_state = State()
        self.current_global_pose = NavSatFix()

        # waypoint를 저장하기 위한 리스트
        self.waypoints = []
        self.path = []

        # Subscriber 및 Publisher 초기화
        self.state_sub = rospy.Subscriber(self.namespace + '/mavros/state', State, self.state_cb)
        self.global_pose_sub = rospy.Subscriber(self.namespace + '/mavros/global_position/global', NavSatFix, self.global_pose_cb)
        self.global_pose_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

        self.waypoints_sub = rospy.Subscriber(self.namespace + '/mavros/mission/waypoints', WaypointList, self.waypoint_cb)
        self.arming_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        self.land_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)

    def state_cb(self, msg):
        rospy.loginfo("Received state message")
        self.current_state = msg

    def global_pose_cb(self, msg):
        self.current_global_pose = msg

    # 웨이포인트 정보 받기
    def waypoint_cb(self, data):
        self.waypoints = data.waypoints

    def takeoff(self, altitude=5.0):
        print("Namespace : " + self.namespace)
        if not self.current_state:
            rospy.logwarn("Current state not yet received!")
            return
        
        if self.current_state.mode != "GUIDED":
            self.set_mode("GUIDED")
        print("guided mode changed")

        self.arm()
        takeoff_cmd = CommandTOL()
        takeoff_cmd.altitude = altitude
        self.takeoff_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=altitude)

    def land(self):
        self.set_mode("LAND")

    def set_mode(self, mode):
        rospy.wait_for_service(self.namespace + '/mavros/set_mode')
        try:
            self.set_mode_client(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def arm(self):
        rospy.wait_for_service(self.namespace + '/mavros/cmd/arming', timeout=5)
        try:
            self.arming_client(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def plan_path_with_astar(self):
        start_global = (self.waypoints[0].x_lat, self.waypoints[0].y_long)
        end_global = (self.waypoints[-1].x_lat, self.waypoints[-1].y_long)

        print(start_global)
        print(end_global)
        
        # 웨이포인트의 첫 지점과 마지막 지점을 출발지, 목적지로 설정 후 astar 알고리즘 경로 찾기
        self.path = astar_main(self.namespace, start_global, end_global)

    def publish_waypoints(self):
        # 경로가 없을 경우
        if not self.path:
            rospy.logerr("Path is empty or None. Cannot publish waypoints.")
            return
        
        rate = rospy.Rate(10)  # 10hz
        for position in self.path:
            waypoint = GlobalPositionTarget()
            waypoint.latitude = position[0]
            waypoint.longitude = position[1]

            waypoint.coordinate_frame = 6

            self.global_pose_pub.publish(waypoint)
            rate.sleep()  # 10Hz로 웨이포인트 발행

    # waypoints에 따른 미션 서비스 클라이언트 생성
    def send_mavros_mission(waypoints):
        try:
            service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            response = service(0, waypoints) # 0 is the starting waypoint
            if not response.success:
                rospy.logerr("Failed to send waypoints to MAVROS")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    # mission_mode로 변경하는 함수
    def set_mission_mode():
        try:
            set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
            response = set_mode_service(0, "AUTO.MISSION")
            if not response.success:
                rospy.logerr("Failed to set AUTO.MISSION mode")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False
    
# 실행 파일로 실행될 경우 name은 main으로 설정
# import 파일로 사용되어질 경우에는 name이 해당 스크립트 파일 명으로 변경
if __name__ == '__main__':
    # ROS 노드 초기화
    # multi_drone_node라는 노드 생성
    rospy.init_node('multi_drone_node', anonymous=True)

    # namespace를 파라미터로 받기
    drone_namespaces = rospy.get_param("~namespaces", "").split(',')
    print("Namespaces value from parameter server:", rospy.get_param("~namespaces", "Default value"))

    print("드론 객체 생성 완료" + ', '.join(drone_namespaces))

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
        
        mavros_waypoints = []
        for position in drone_node.path:
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL
            wp.command = Waypoint.NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = 0
            wp.param2 = 0
            wp.param3 = 0
            wp.param4 = 0
            wp.x_lat = position[0]
            wp.y_long = position[1]
            wp.z_alt = 20.0  # altitude (고도)는 필요에 따라 조정
            mavros_waypoints.append(wp)

        if drone_node.send_mavros_mission(mavros_waypoints):
            drone_node.set_mission_mode()
    
    # ROS 노드를 실행 상태로 유지하며 메시지 계속 수신하도록 함
    rospy.spin()
